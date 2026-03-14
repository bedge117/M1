/* See COPYING.txt for license details. */

/*
 ******************************************************************************
 * nfc_listener.c - NFC Listener Mode (Card Emulation)
 ******************************************************************************
 * 
 * [Purpose]
 * - Implements NFC-A Type 2 Tag (T2T) card emulation
 * - Handles T2T commands (READ, GET_VERSION, FAST_READ, WRITE, etc.)
 * - Synchronous transmission for strict timing requirements (FDT: 86~177μs)
 * 
 * [State Machine]
 * NOTINIT → IDLE → START_DISCOVERY → DISCOVERY → DATAEXCHANGE
 * 
 * [Key Flow]
 * 1. ListenIni(): Initialize RFAL in listener mode
 *    - Configure discovery parameters (LISTEN_TECH_A only)
 *    - Set UID, ATQA, SAK for emulation
 *    - Start rfalNfcDiscover()
 * 
 * 2. ListenerCycle(): Main processing loop
 *    - DISCOVERY: Wait for reader field, handle ACTIVATED state
 *    - DATAEXCHANGE: Process T2T commands from reader
 * 
 * 3. ACTIVATED → DATAEXCHANGE Transition:
 *    - rfal_nfc.c receives T2T command in LISTEN_ACTIVATED state
 *    - Data stored in gNfcDev.rxBuf.rfBuf
 *    - rfalNfcDataExchangeStart() retrieves already-received data pointer
 *    - rfalNfcDataExchangeGetStatus() checks if data exists
 *    - Transition to DATAEXCHANGE state
 * 
 * 4. T2T Command Processing (CeHandleT2TCmdRx()):
 *    - 0x30 (READ): Build page response, sync TX (500μs FWT)
 *    - 0x3A (FAST_READ): Build multi-page response, sync TX
 *    - 0x60 (GET_VERSION): Send version info, sync TX
 *    - 0xA2 (WRITE): Save page data, send ACK
 *    - After TX: Re-arm RX for next command
 * 
 * [Timing Critical]
 * - Uses rfalTransceiveBlockingTx() for immediate response
 * - FWT = 500μs (sufficient margin for T2T standard 86~177μs FDT)
 * - RX re-arm immediately after TX completion
 * 
 * [Phase States]
 * - CE_PHASE_WAIT_RX: Waiting for command from reader
 * - CE_PHASE_DATAEX: Data exchange in progress (TX completed)
 * 
 ******************************************************************************
 */
#include "nfc_poller.h"
#include "utils.h"
#include "rfal_nfc.h"
#include "rfal_t2t.h"
#include "rfal_utils.h"
#include "logger.h"

#include "st25r3916.h"
#include "st25r3916_com.h"
#include "rfal_utils.h"

#include "rfal_analogConfig.h"
#include "rfal_rf.h"
#include "uiView.h"
#include "nfc_driver.h"
#include "nfc_listener.h"
#include "common/nfc_ctx.h"
#include "lfrfid.h"
#include "rfal_nfc.h"
#include "mfc_crypto1.h"

#define NOTINIT             0     
#define IDLE                1     
#define START_DISCOVERY     3    
#define DISCOVERY           4    
#define SELECT              5     
#define DATAEXCHANGE        6     

uint8_t g_T2tCmd;

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
static uint16_t g_ceTxLenBytes = 0U;
static bool     g_ceTxPending  = false;

static rfalNfcDiscoverParam discParam;
static uint8_t              state = NOTINIT;
static bool                 multiSel;

static EmuPersona_t g_persona = EMU_PERSONA_RAW;
void Emu_SetPersona(EmuPersona_t p) { g_persona = p; }
EmuPersona_t Emu_GetPersona(void)   { return g_persona; }
static uint16_t ceT2T_FromDump(const uint8_t *rx, uint16_t rxLenBytes, uint8_t *tx, uint16_t txMaxLen);

/* CE state and buffer */
typedef enum {
    CE_PHASE_IDLE = 0,
    CE_PHASE_WAIT_RX,
    CE_PHASE_WAIT_TX,
    CE_PHASE_DATAEX,   /* Data exchange in progress */
} CePhase_t;

static rfalNfcDevice *s_ceActiveDev = NULL;
static uint8_t       *s_ceRxData    = NULL;
static uint16_t      *s_ceRxRcvLen  = NULL;
static uint8_t        g_ceTxBuf[192];
static bool s_firstRxHandled = false;
static CePhase_t     s_cePhase   = CE_PHASE_IDLE;
static uint8_t       *s_rxData    = NULL;   // Buffer pointer managed by RFAL
static uint16_t      *s_rcvLen    = NULL;
static rfalNfcDevice *s_active_dev = NULL;
static volatile bool s_listener_stop = false;
static bool          s_wasActivated = false;
static bool          s_firstRxPending = false;

static uint16_t s_lastRxBits;
static uint8_t  s_lastRxBuf[32]; /* As needed length */


/*
 ******************************************************************************
 * MIFARE Classic Card Emulation State
 ******************************************************************************
 */
typedef enum {
    MFC_CE_IDLE = 0,
    MFC_CE_SENT_NT,   /* Sent nT, waiting for reader's {nR, aR} */
    MFC_CE_AUTHED,    /* Authenticated — subsequent commands are encrypted */
} MfcCePhase_t;

static struct {
    crypto1_state_t cs;
    MfcCePhase_t    phase;
    uint32_t        nT;
    uint32_t        uid32;
} s_mfcCe;

/* Determine sector number from block number */
static uint16_t ceMfc_SectorOfBlock(uint16_t blockNo)
{
    if (blockNo < 128) return blockNo / 4;
    return (uint16_t)(32 + (blockNo - 128) / 16);
}

/* Get sector trailer block number */
static uint16_t ceMfc_TrailerOfSector(uint16_t sector)
{
    if (sector < 32) return (uint16_t)(sector * 4 + 3);
    return (uint16_t)(128 + (sector - 32) * 16 + 15);
}

/* Extract key from dump sector trailer for a given block */
static bool ceMfc_GetKeyForBlock(uint8_t blockNo, uint8_t keyType, uint8_t key[6])
{
    nfc_run_ctx_t *c = nfc_ctx_get();
    if (!c || !c->dump.has_dump || !c->dump.data) return false;
    if (c->dump.unit_size != MFC_BLOCK_SIZE) return false;

    uint16_t sector  = ceMfc_SectorOfBlock(blockNo);
    uint16_t trailer = ceMfc_TrailerOfSector(sector);

    if (trailer >= c->dump.unit_count) return false;

    uint8_t *td = &c->dump.data[trailer * MFC_BLOCK_SIZE];
    if (keyType == MFC_CMD_AUTH_A)
        memcpy(key, &td[0], MFC_KEY_LEN);   /* Key A at offset 0 */
    else
        memcpy(key, &td[10], MFC_KEY_LEN);  /* Key B at offset 10 */
    return true;
}

/* Get block data from dump */
static bool ceMfc_GetBlockData(uint8_t blockNo, uint8_t out[MFC_BLOCK_SIZE])
{
    nfc_run_ctx_t *c = nfc_ctx_get();
    if (!c || !c->dump.has_dump || !c->dump.data) return false;
    if (blockNo >= c->dump.unit_count) return false;

    memcpy(out, &c->dump.data[(uint32_t)blockNo * MFC_BLOCK_SIZE], MFC_BLOCK_SIZE);
    return true;
}

/* Compute ISO14443A CRC over buf[0..len-1] */
static uint16_t ceMfc_CrcA(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0x6363;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t bt = buf[i];
        bt ^= (uint8_t)(crc & 0xFF);
        bt ^= (bt << 4);
        crc = (crc >> 8) ^ ((uint16_t)bt << 8) ^ ((uint16_t)bt << 3) ^ ((uint16_t)bt >> 4);
    }
    return crc;
}

/*============================================================================*/
/* CeHandleMfcCmdRx — Handle MIFARE Classic command in CE mode               */
/*                                                                            */
/* State machine:                                                             */
/* MFC_CE_IDLE    → Receive AUTH cmd → send nT → MFC_CE_SENT_NT              */
/* MFC_CE_SENT_NT → Receive {nR,aR}  → verify  → send aT → MFC_CE_AUTHED   */
/* MFC_CE_AUTHED  → Receive encrypted cmd → process → send encrypted response*/
/*============================================================================*/
static bool CeHandleMfcCmdRx(const uint8_t *rx, uint16_t rxBits)
{
    if (!rx || rxBits < 8) return false;
    uint16_t rxBytes = rfalConvBitsToBytes(rxBits);
    ReturnCode err;

    switch (s_mfcCe.phase) {

    case MFC_CE_IDLE:
    {
        /* Expecting AUTH command: 0x60 or 0x61 + blockNo (CRC stripped by RFAL) */
        if (rxBytes < 2) return false;
        uint8_t cmd     = rx[0];
        uint8_t blockNo = rx[1];

        if (cmd != MFC_CMD_AUTH_A && cmd != MFC_CMD_AUTH_B) {
            /* Not an AUTH — might be HALT or unexpected */
            return false;
        }

        uint8_t key[MFC_KEY_LEN];
        if (!ceMfc_GetKeyForBlock(blockNo, cmd, key)) {
            platformLog("[CE-MFC] no key for block %u\r\n", blockNo);
            return false;
        }

        /* Generate nT from HAL tick */
        extern uint32_t HAL_GetTick(void);
        s_mfcCe.nT = HAL_GetTick() * 0x19660D + 0x3C6EF35F;

        /* Initialize Crypto-1 cipher */
        uint64_t key64 = 0;
        for (int i = 0; i < MFC_KEY_LEN; i++)
            key64 = (key64 << 8) | key[i];
        crypto1_init(&s_mfcCe.cs, key64);

        /* Feed uid XOR nT into cipher to set up state */
        crypto1_word(&s_mfcCe.cs, s_mfcCe.uid32 ^ s_mfcCe.nT, 0);

        /* Send nT (4 bytes, no CRC, standard parity — correct for this frame) */
        uint8_t nTbuf[4];
        nTbuf[0] = (uint8_t)(s_mfcCe.nT >> 24);
        nTbuf[1] = (uint8_t)(s_mfcCe.nT >> 16);
        nTbuf[2] = (uint8_t)(s_mfcCe.nT >> 8);
        nTbuf[3] = (uint8_t)(s_mfcCe.nT);

        err = rfalTransceiveBlockingTx(
            nTbuf, sizeof(nTbuf), NULL, 0, NULL,
            (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL,
            rfalConvUsTo1fc(500U));

        if (err != RFAL_ERR_NONE) {
            platformLog("[CE-MFC] nT TX err=%d\r\n", err);
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }

        s_mfcCe.phase = MFC_CE_SENT_NT;

        /* Re-arm RX */
        err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
        if (err == RFAL_ERR_NONE) {
            s_cePhase = CE_PHASE_WAIT_RX;
        }
        platformLog("[CE-MFC] AUTH block %u: sent nT\r\n", blockNo);
        return true;
    }

    case MFC_CE_SENT_NT:
    {
        /* Expecting encrypted {nR, aR} — 8 bytes from reader */
        if (rxBytes < 8) {
            platformLog("[CE-MFC] {nR,aR} too short: %u bytes\r\n", rxBytes);
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }

        /* Decrypt nR: keystream = crypto1_word(enR, is_encrypted=1) */
        uint32_t enR = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
                       ((uint32_t)rx[2] << 8)  | (uint32_t)rx[3];
        uint32_t ks1 = crypto1_word(&s_mfcCe.cs, enR, 1);
        (void)ks1; /* nR = enR ^ ks1, but we don't need nR itself */

        /* Decrypt aR */
        uint32_t eaR = ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16) |
                       ((uint32_t)rx[6] << 8)  | (uint32_t)rx[7];
        uint32_t ks2 = crypto1_word(&s_mfcCe.cs, 0, 0);
        uint32_t aR  = eaR ^ ks2;

        /* Verify aR == suc64(nT) */
        uint32_t expected_aR = mfc_prng_successor(s_mfcCe.nT, 64);
        if (aR != expected_aR) {
            platformLog("[CE-MFC] auth verify FAILED\r\n");
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }

        /* Compute and send encrypted aT = suc96(nT) */
        uint32_t aT  = mfc_prng_successor(s_mfcCe.nT, 96);
        uint32_t ks3 = crypto1_word(&s_mfcCe.cs, 0, 0);
        uint32_t eaT = aT ^ ks3;

        uint8_t aTbuf[4];
        aTbuf[0] = (uint8_t)(eaT >> 24);
        aTbuf[1] = (uint8_t)(eaT >> 16);
        aTbuf[2] = (uint8_t)(eaT >> 8);
        aTbuf[3] = (uint8_t)(eaT);

        err = rfalTransceiveBlockingTx(
            aTbuf, sizeof(aTbuf), NULL, 0, NULL,
            (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL,
            rfalConvUsTo1fc(500U));

        if (err != RFAL_ERR_NONE) {
            platformLog("[CE-MFC] aT TX err=%d\r\n", err);
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }

        s_mfcCe.phase = MFC_CE_AUTHED;

        /* Re-arm RX */
        err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
        if (err == RFAL_ERR_NONE) {
            s_cePhase = CE_PHASE_WAIT_RX;
        }
        platformLog("[CE-MFC] auth OK → AUTHED\r\n");
        return true;
    }

    case MFC_CE_AUTHED:
    {
        /* All commands are encrypted. Decrypt first. */
        if (rxBytes < 4) {
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }

        /* Decrypt received command (4 bytes: cmd + arg + CRC) */
        uint8_t plain[4];
        for (int i = 0; i < 4 && i < (int)rxBytes; i++) {
            uint8_t ks = crypto1_byte(&s_mfcCe.cs, rx[i], 1);
            plain[i] = rx[i] ^ ks;
        }

        uint8_t cmd = plain[0];

        if (cmd == MFC_CMD_READ) {
            /* READ command: send encrypted block data + CRC */
            uint8_t blockNo = plain[1];
            uint8_t blockData[MFC_BLOCK_SIZE];

            if (!ceMfc_GetBlockData(blockNo, blockData)) {
                memset(blockData, 0x00, MFC_BLOCK_SIZE);
            }

            /* Append CRC to block data */
            uint8_t resp[18]; /* 16 data + 2 CRC */
            memcpy(resp, blockData, MFC_BLOCK_SIZE);
            uint16_t crc = ceMfc_CrcA(resp, MFC_BLOCK_SIZE);
            resp[16] = (uint8_t)(crc & 0xFF);
            resp[17] = (uint8_t)(crc >> 8);

            /* Encrypt the response */
            uint8_t enc[18];
            for (int i = 0; i < 18; i++) {
                uint8_t ks = crypto1_byte(&s_mfcCe.cs, 0, 0);
                enc[i] = resp[i] ^ ks;
            }

            err = rfalTransceiveBlockingTx(
                enc, sizeof(enc), NULL, 0, NULL,
                (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL,
                rfalConvUsTo1fc(500U));

            if (err != RFAL_ERR_NONE && err != RFAL_ERR_LINK_LOSS) {
                platformLog("[CE-MFC] READ TX err=%d\r\n", err);
                s_mfcCe.phase = MFC_CE_IDLE;
                return false;
            }

            /* Re-arm RX */
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return true;
        }
        else if (cmd == MFC_CMD_AUTH_A || cmd == MFC_CMD_AUTH_B) {
            /* Re-authentication for a different sector */
            s_mfcCe.phase = MFC_CE_IDLE;
            /* Re-process this command as a new AUTH */
            return CeHandleMfcCmdRx(plain, rfalConvBytesToBits(4));
        }
        else if (cmd == MFC_CMD_HALT) {
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }
        else {
            /* Unknown encrypted command — send NACK */
            platformLog("[CE-MFC] unknown encrypted cmd 0x%02X\r\n", cmd);
            s_mfcCe.phase = MFC_CE_IDLE;
            return false;
        }
    }

    default:
        s_mfcCe.phase = MFC_CE_IDLE;
        return false;
    }
}

__attribute__((weak)) void Listener_OnActivated(const rfalNfcDevice* dev) { (void)dev; }
__attribute__((weak)) void Listener_OnDeactivated(void) {}

static void CeApplyDiscParamToActiveDev( rfalNfcDevice *dev );
//static uint16_t CeBuildEmulationResponse( const uint8_t *rx, uint16_t rxLenB, uint8_t *tx, uint16_t txSize );
static ReturnCode CeArmRx(void);
static void ListenerNotif( rfalNfcState st );


/*============================================================================*/
/**
 * @brief ListenerGetLastRx - Get last received data for listener
 * 
 * @param[out] lenBits Pointer to store received length in bits
 * @retval Pointer to last received data buffer, or NULL if no data
 */
/*============================================================================*/
const uint8_t* ListenerGetLastRx(uint16_t *lenBits)
{
    if (lenBits) *lenBits = s_lastRxBits;
    return s_lastRxBits ? s_lastRxBuf : NULL;
}

/*============================================================================*/
/**
 * @brief ListenerGetActiveDev - Get active device for listener
 * 
 * @retval Pointer to active NFC device, or NULL if none
 */
/*============================================================================*/
const rfalNfcDevice* ListenerGetActiveDev(void)
{
    return s_active_dev;
}

/*============================================================================*/
/**
 * @brief ListenerRequestStop - Request CE stop from external
 * 
 * @retval None
 */
/*============================================================================*/
void ListenerRequestStop(void) { s_listener_stop = true; }

/*============================================================================*/
/**
 * @brief CeArmRx - Arm RFAL RX for the next frame (LISTEN/T2T)
 * 
 * Arms the RFAL receiver to wait for the next command from reader.
 * 
 * @retval RFAL_ERR_NONE Success
 * @retval Other RFAL error codes
 */
/*============================================================================*/
static ReturnCode CeArmRx(void)
{
    s_ceRxData   = NULL;
    s_ceRxRcvLen = NULL;
    ReturnCode err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
    platformLog("[CE] Arm RX err=%d\r\n", err);
    return err;
}

static void CeApplyDiscParamToActiveDev( rfalNfcDevice *dev )
{
    if (dev == NULL) {
        return;
    }

    dev->type = RFAL_NFC_LISTEN_TYPE_NFCA;

    uint8_t uidLen = (discParam.lmConfigPA.nfcidLen == RFAL_LM_NFCID_LEN_07) ? 7U : 4U;

    if (dev->nfcid != NULL) {
        dev->nfcidLen = uidLen;
        ST_MEMCPY(dev->nfcid, discParam.lmConfigPA.nfcid, uidLen);
    }

    dev->dev.nfca.nfcId1Len = uidLen;
    ST_MEMCPY(dev->dev.nfca.nfcId1, discParam.lmConfigPA.nfcid, uidLen);

    dev->dev.nfca.sensRes.anticollisionInfo = discParam.lmConfigPA.SENS_RES[0];
    dev->dev.nfca.sensRes.platformInfo      = discParam.lmConfigPA.SENS_RES[1];
    dev->dev.nfca.selRes.sak                = discParam.lmConfigPA.SEL_RES;
    dev->dev.nfca.type = (g_persona == EMU_PERSONA_RAW) ? RFAL_NFCA_T2T : RFAL_NFCA_T4T;
    dev->dev.nfca.isSleep = false;
    platformLog("[CE] Applied Discovery Param to Active Dev\r\n");
}

/*============================================================================*/
/**
 * @brief CeBuildEmulationResponse - Build emulation response based on received frame
 * 
 * Receives RX frame and builds appropriate emulation response based on emulation type.
 * 
 * @param[in] rx Received data buffer
 * @param[in] rxLenB Received data length in bytes
 * @param[out] tx Transmission buffer
 * @param[in] txSize Transmission buffer size
 * @retval Response length in bytes, or 0 on error
 */
/*============================================================================*/
static uint16_t __attribute__((unused)) CeBuildEmulationResponse( const uint8_t *rx, uint16_t rxLenB, uint8_t *tx, uint16_t txSize )
{
    if (!rx || rxLenB == 0U || !tx || txSize == 0U) {
        platformLog("[CE][DBG] CeBuildEmulationResponse: invalid args rx=%p rxLenB=%u tx=%p txSize=%u\r\n",
                    rx, (unsigned)rxLenB, tx, (unsigned)txSize);
        return 0;
    }

    nfc_run_ctx_t *c = nfc_ctx_get();
    if (!c) {
        platformLog("[CE][DBG] CeBuildEmulationResponse: no context\r\n");
        return 0;
    }

    /* Currently only NFC-A based emulation is handled */
    if (c->head.tech != M1NFC_TECH_A) {
        platformLog("[CE][DBG] CeBuildEmulationResponse: unsupported tech=%d\r\n", c->head.tech);
        return 0;
    }

    switch (c->head.family) {
    case M1NFC_FAM_CLASSIC:
        /* Classic emulation handled by CeHandleMfcCmdRx via persona dispatch */
        platformLog("[CE][DBG] Classic: use persona-based dispatch\r\n");
        return 0;

    case M1NFC_FAM_ULTRALIGHT:
        return ceT2T_FromDump(rx, rxLenB, tx, txSize);
    case M1NFC_FAM_DESFIRE:
        /* TODO: Replace with ceDesfire_FromDump when implemented */
        if (txSize >= 2) { tx[0] = 0x90; tx[1] = 0x00; return 2; }
        platformLog("[CE][DBG] Desfire not implemented\r\n");
        return 0;

    default:
        platformLog("[CE][DBG] Unknown family=%d\r\n", c->head.family);
        return 0;
    }

    return 0;
}

/*============================================================================*/
/**
 * @brief SendGetVersion - Handle GET_VERSION command (send response if cmd==0x60, else return false)
 * 
 * @param[in] rx Received data buffer
 * @param[in] rxLenB Received data length in bytes
 * @retval true If GET_VERSION command was handled
 * @retval false If not GET_VERSION command
 */
/*============================================================================*/
bool SendGetVersion(const uint8_t *rx, uint16_t rxLenB)
{
    if (!rx || rxLenB < 1U || rx[0] != 0x60) {
        return false;
    }

    /* NTAG215 example version */
    uint8_t ver[8] = {0x00, 0x04, 0x04, 0x02, 0x01, 0x00, 0x11, 0x03};
    platformLog("[CE][TX] GET_VERSION rsp: %s\r\n", hex2Str(ver, sizeof(ver)));

    s_firstRxHandled = true;
    rfalNfcDataExchangeStart(ver, rfalConvBytesToBits(sizeof(ver)), &s_rxData, &s_rcvLen, RFAL_FWT_NONE);
    return true;
}

/*============================================================================*/
/**
 * @brief CeBuildT2TReadResp - Build T2T READ response
 * 
 * Builds response for T2T READ command (0x30) by reading 4 pages from context.
 * 
 * @param[in] rx Received data buffer
 * @param[in] rxBits Received data length in bits
 * @retval RFAL_ERR_NONE Success
 * @retval RFAL_ERR_PARAM Invalid parameters
 */
/*============================================================================*/
static ReturnCode CeBuildT2TReadResp(const uint8_t *rx, uint16_t rxBits)
{
    if (!rx || rxBits < 16) return RFAL_ERR_PARAM;

    uint16_t rxBytes = rfalConvBitsToBytes(rxBits);
    if (rxBytes < 2) return RFAL_ERR_PARAM;

    uint8_t startPage = rx[1];
    uint16_t txLen = 0;

    /* Prepare 4 pages of data (g_ceTxBuf is already initialized externally) */
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t page = (uint16_t)startPage + i;
        uint16_t bufOffset = txLen;
        
        if (!nfc_ctx_get_t2t_page(page, &g_ceTxBuf[bufOffset])) {
            /* Fill with 0 if page doesn't exist */
            g_ceTxBuf[bufOffset]     = 0x00;
            g_ceTxBuf[bufOffset + 1] = 0x00;
            g_ceTxBuf[bufOffset + 2] = 0x00;
            g_ceTxBuf[bufOffset + 3] = 0x00;
        }
        txLen += 4;
    }

    g_ceTxLenBytes = (uint16_t)txLen;
    return RFAL_ERR_NONE;
}


/*============================================================================*/
/**
 * @brief CeHandleT2TCmdRx - Dispatch T2T commands
 * 
 * Handles T2T commands from reader:
 * - 0x30 (READ): Read 4 pages
 * - 0x3A (FAST_READ): Read multiple pages
 * - 0x60 (GET_VERSION): Send version info
 * - 0xA2 (WRITE): Write one page
 * - Others: Simple ACK responses
 * 
 * @param[in] rx Received data buffer
 * @param[in] rxBits Received data length in bits
 * @retval true If command was handled successfully
 * @retval false If command was not handled or error occurred
 */
/*============================================================================*/
static bool CeHandleT2TCmdRx(const uint8_t *rx, uint16_t rxBits)
{
    if (!rx || rxBits < 8) return false;
    
    uint16_t rxBytes = rfalConvBitsToBytes(rxBits);
    if (rxBytes < 1) return false;
    
    uint8_t cmd = rx[0];
    ReturnCode err;
    //uint32_t fwt;
    
    switch (cmd) {
        case 0x30: {
            /* READ command: Prepare response then start TX immediately */
            uint8_t startPage = (rxBytes >= 2) ? rx[1] : 0;
            
            err = CeBuildT2TReadResp(rx, rxBits);
            if (err != RFAL_ERR_NONE || g_ceTxLenBytes == 0U) {
                return false;
            }
            
            /* Log only critical page reads (0, 4, 8, etc.) */
            if ((startPage % 8) == 0) {
                //platformLog("[CE] READ p%u: %s\r\n", startPage, hex2Str(g_ceTxBuf, 8));
            }
            
            /* Immediate synchronous transmission: Meet T2T FDT 86~177μs requirement */
            err = rfalTransceiveBlockingTx(g_ceTxBuf, g_ceTxLenBytes, NULL, 0, NULL, RFAL_TXRX_FLAGS_DEFAULT, rfalConvUsTo1fc(500U));
            if (err != RFAL_ERR_NONE && err != RFAL_ERR_LINK_LOSS) {
                platformLog("[CE] READ TX err=%d\r\n", err);
                return false;
            }
            
            /* TX complete, re-arm RX (wait for next command) */
            g_ceTxPending = false;
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return (err == RFAL_ERR_NONE);
        }
        
        case 0x3A: {
            /* FAST_READ command: Read multiple pages sequentially (start ~ end) */
            if (rxBytes < 3) {
                return false;
            }
            
            uint8_t startPage = rx[1];
            uint8_t endPage   = rx[2];
            uint16_t pageCnt = nfc_ctx_get_t2t_page_count();
            
            if (endPage < startPage) {
                return false;
            }
            
            if (endPage >= pageCnt) {
                endPage = (uint8_t)(pageCnt - 1);
            }
            
            uint16_t numPages = (uint16_t)(endPage - startPage + 1);
            uint16_t txLen = 0;
            uint8_t pageBuf[4];
            
            for (uint16_t i = 0; i < numPages; i++) {
                uint16_t page = (uint16_t)startPage + i;
                if (!nfc_ctx_get_t2t_page(page, pageBuf)) {
                    pageBuf[0] = pageBuf[1] = pageBuf[2] = pageBuf[3] = 0x00;
                }
                
                if (txLen + 4U > sizeof(g_ceTxBuf)) break;
                memcpy(&g_ceTxBuf[txLen], pageBuf, 4);
                txLen += 4;
            }
            
            /* Immediate synchronous transmission */
            err = rfalTransceiveBlockingTx(g_ceTxBuf, txLen, NULL, 0, NULL, RFAL_TXRX_FLAGS_DEFAULT, rfalConvUsTo1fc(500U));
            if (err != RFAL_ERR_NONE && err != RFAL_ERR_LINK_LOSS) {
                return false;
            }
            
            /* TX complete, re-arm RX */
            g_ceTxPending = false;
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return (err == RFAL_ERR_NONE);
        }
        
        case 0x60: {
            /* GET_VERSION command: NTAG/Ultralight-C series */
            uint8_t ver[8];
            if (!nfc_ctx_get_t2t_version(ver)) {
                platformLog("[CE] GET_VER: no data\r\n");
                return false;
            }
            
            platformLog("[CE] GET_VER TX: %s\r\n", hex2Str(ver, 8));
            
            /* Immediate synchronous transmission: Meet T2T FDT 86~177μs requirement */
            err = rfalTransceiveBlockingTx(ver, sizeof(ver), NULL, 0, NULL, RFAL_TXRX_FLAGS_DEFAULT, rfalConvUsTo1fc(500U));
            if (err != RFAL_ERR_NONE) {
                platformLog("[CE] GET_VER TX err=%d\r\n", err);
                return false;
            }
            
            /* TX complete, re-arm RX (wait for next command) */
            g_ceTxPending = false;
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return (err == RFAL_ERR_NONE);
        }
        
        case 0xA2: {
            /* WRITE command: Write one page (4 bytes) */
            if (rxBytes < 6) {
                return false;
            }
            
            uint8_t page = rx[1];
            uint8_t data[4];
            memcpy(data, &rx[2], 4);
            
            /* Save page data */
            nfc_ctx_set_t2t_page(page, data);
            
            /* WRITE response: ACK (0x0A), immediate synchronous transmission */
            g_ceTxBuf[0] = 0x0A;
            err = rfalTransceiveBlockingTx(g_ceTxBuf, 1, NULL, 0, NULL, RFAL_TXRX_FLAGS_DEFAULT, rfalConvUsTo1fc(500U));
            if (err != RFAL_ERR_NONE && err != RFAL_ERR_LINK_LOSS) {
                return false;
            }
            
            g_ceTxPending = false;
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return (err == RFAL_ERR_NONE);
        }
        
        case 0xC2:
        case 0x1B:
        case 0x39:
        case 0x53:
        case 0x57:
        case 0xE0: {
            /* Simple response commands: ACK or fixed data */
            uint8_t respLen = 1;  /* Default ACK */
            g_ceTxBuf[0] = 0x0A;  /* ACK */
            
            if (cmd == 0x1B) {
                /* READ_CNT: 3-byte counter */
                g_ceTxBuf[0] = g_ceTxBuf[1] = g_ceTxBuf[2] = 0x00;
                respLen = 3;
            } else if (cmd == 0x39) {
                /* READ_SIG: 32-byte signature */
                memset(g_ceTxBuf, 0x00, 32);
                respLen = 32;
            } else if (cmd == 0x53) {
                /* PWD_AUTH: PACK response */
                g_ceTxBuf[0] = 0x80;
            }
            
            /* Immediate synchronous transmission */
            err = rfalTransceiveBlockingTx(g_ceTxBuf, respLen, NULL, 0, NULL, RFAL_TXRX_FLAGS_DEFAULT, rfalConvUsTo1fc(500U));
            if (err != RFAL_ERR_NONE && err != RFAL_ERR_LINK_LOSS) {
                return false;
            }
            
            g_ceTxPending = false;
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err == RFAL_ERR_NONE) {
                s_cePhase = CE_PHASE_WAIT_RX;
            }
            return (err == RFAL_ERR_NONE);
        }
        
        default:
            return false;           
    }
}


/*============================================================================*/
/**
 * @brief ListenIni - Initialize NFC listener (CARD EMULATION / LISTENER) Set Profile
 * 
 * Initializes RFAL in listener mode for card emulation.
 * Configures discovery parameters, sets UID/ATQA/SAK for emulation,
 * and starts discovery process.
 * 
 * @retval true Initialization successful
 * @retval false Initialization failed
 */
/*============================================================================*/
bool ListenIni(void)
{
    ReturnCode err = RFAL_ERR_NONE;

    /* --- Global CE state reset --- */
    s_cePhase       = CE_PHASE_IDLE;
    s_rxData        = NULL;
    s_rcvLen        = NULL;
    s_active_dev    = NULL;
    s_listener_stop = false;
    s_wasActivated  = false;
    s_firstRxPending = false;
    state           = IDLE;

    for (int i = 0; i < 2; i++) {
        err = rfalNfcInitialize();
        //platformLog("rfalNfcInitialize() = %d\r\n", err);
        if (err == RFAL_ERR_NONE) break;
        vTaskDelay(5);
    }
    if (err != RFAL_ERR_NONE) return false;

    /* 2) Discovery parameters: Listener(CE) only */
    rfalNfcDefaultDiscParams(&discParam);

    // --- CE-only safety guard (remove POLL traces/garbage values) ---
    discParam.maxBR  = RFAL_BR_KEEP;

    discParam.GBLen  = 0;
    discParam.ap2pBR = RFAL_BR_106;     // Safe default value even if not used
    discParam.nfcfBR = RFAL_BR_212;     // Safe even if only LISTEN_F is enabled, compared to POLL_F

    // Remove all POLL bits and set only LISTEN
    discParam.techs2Find &= ~( RFAL_NFC_POLL_TECH_A | RFAL_NFC_POLL_TECH_B |
                            RFAL_NFC_POLL_TECH_F | RFAL_NFC_POLL_TECH_V |
                            RFAL_NFC_POLL_TECH_AP2P | RFAL_NFC_POLL_TECH_ST25TB );

    discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;

    discParam.devLimit       = 1U;
    discParam.totalDuration  = 60000U;                 /* CE waits longer (e.g., 60s) */
    discParam.notifyCb       = ListenerNotif;
#if defined(RFAL_COMPLIANCE_MODE_NFC)
    discParam.compMode       = RFAL_COMPLIANCE_MODE_NFC;
#endif

    /* Turn off all Poller, enable only Listen */
    discParam.techs2Find     = RFAL_NFC_TECH_NONE;
    discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;   /* CE-A active */

    EmuNfcA_t emuA;
    bool has_ctx = Emu_GetNfcA(&emuA);
    const uint8_t default_uid[7] = { 0x08, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

    /* UID fill (fallback to default when no emu context) */
    if (has_ctx) {
        discParam.lmConfigPA.nfcidLen = (emuA.uid_len == 7)
                                      ? RFAL_LM_NFCID_LEN_07
                                      : RFAL_LM_NFCID_LEN_04;
        ST_MEMCPY(discParam.lmConfigPA.nfcid, emuA.uid,
                  (discParam.lmConfigPA.nfcidLen == RFAL_LM_NFCID_LEN_07) ? 7 : 4);
    } else {
        discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;
        ST_MEMCPY(discParam.lmConfigPA.nfcid, default_uid, 4);
        platformLog("[CE] No emu context; using default UID\r\n");
    }

    /* Fill ATQA/SAK + auto-infer persona */
    if (has_ctx) {
        /* Check NFC context family and force T2T if applicable */
        nfc_run_ctx_t* ctx = nfc_ctx_get();
        bool is_t2t_family = (ctx && ctx->head.family == M1NFC_FAM_ULTRALIGHT);
        
        if (is_t2t_family) {
            /* If T2T family: Force SAK=0x00, ATQA=0x44 0x00 (prevent ISO-DEP) */
            discParam.lmConfigPA.SENS_RES[0] = 0x44;
            discParam.lmConfigPA.SENS_RES[1] = 0x00;
            discParam.lmConfigPA.SEL_RES     = 0x00;
            g_persona = EMU_PERSONA_T2T;
            platformLog("[CE] T2T family: forcing ATQA=0x4400 SAK=0x00 (ISO-DEP disabled)\r\n");
        } else {
            /* Apply context values as-is */
            discParam.lmConfigPA.SENS_RES[0] = emuA.atqa[0];
            discParam.lmConfigPA.SENS_RES[1] = emuA.atqa[1];
            discParam.lmConfigPA.SEL_RES     = emuA.sak;

            /* Infer persona based on ATQA/SAK */
            if (emuA.sak == 0x20) {
                /* Type 4A (ISO-DEP / DESFire series) */
                g_persona = EMU_PERSONA_T4T;
                platformLog("[CE] Persona=T4T ATQA=%02X%02X SAK=%02X\r\n", emuA.atqa[0], emuA.atqa[1], emuA.sak);
                discParam.lmConfigPA.SEL_RES     = 0x00; // Force SAK 0x00 for T4T compatibility with common NFC readers (temporary)
            } else if (emuA.sak == 0x08 || emuA.sak == 0x18 || emuA.sak == 0x09 ||
                       emuA.sak == 0x01 || emuA.sak == 0x10 || emuA.sak == 0x11 ||
                       emuA.sak == 0x19 || emuA.sak == 0x28 || emuA.sak == 0x38) {
                /* MIFARE Classic / Plus / EV1 variants */
                g_persona = EMU_PERSONA_MFC;
                /* Initialize MFC CE state */
                memset(&s_mfcCe, 0, sizeof(s_mfcCe));
                s_mfcCe.phase = MFC_CE_IDLE;
                /* Store UID for crypto */
                if (emuA.uid_len >= 7)
                    s_mfcCe.uid32 = ((uint32_t)emuA.uid[3] << 24) | ((uint32_t)emuA.uid[4] << 16) |
                                    ((uint32_t)emuA.uid[5] << 8)  | (uint32_t)emuA.uid[6];
                else
                    s_mfcCe.uid32 = ((uint32_t)emuA.uid[0] << 24) | ((uint32_t)emuA.uid[1] << 16) |
                                    ((uint32_t)emuA.uid[2] << 8)  | (uint32_t)emuA.uid[3];
                platformLog("[CE] Persona=MFC ATQA=%02X%02X SAK=%02X uid32=%08lX\r\n",
                            emuA.atqa[0], emuA.atqa[1], emuA.sak, (unsigned long)s_mfcCe.uid32);
            } else if (emuA.atqa[0] == 0x44 && emuA.atqa[1] == 0x00 && emuA.sak == 0x00) {
                /* NTAG/Ultralight (Type 2) */
                g_persona = EMU_PERSONA_T2T;
                platformLog("[CE] Persona=T2T ATQA=%02X%02X SAK=%02X\r\n", emuA.atqa[0], emuA.atqa[1], emuA.sak);
            } else {
                /* Others: Use ATQA/SAK from read card as-is */
                g_persona = EMU_PERSONA_RAW;
                platformLog("[CE] Persona=RAW ATQA=%02X%02X SAK=%02X\r\n", emuA.atqa[0], emuA.atqa[1], emuA.sak);
            }
        }

    } else {
        /* If no context, force default T2T values */
        discParam.lmConfigPA.SENS_RES[0] = 0x44;
        discParam.lmConfigPA.SENS_RES[1] = 0x00;
        discParam.lmConfigPA.SEL_RES     = 0x00;
        g_persona = EMU_PERSONA_T2T;
        platformLog("[CE] Persona=T2T (default) ATQA=4400 SAK=00\r\n");
    }


    /* Check if T2T dump is ready */
    {
        uint16_t t2_pages = nfc_ctx_get_t2t_page_count();
        if (t2_pages == 0) {
            platformLog("[CE] WARNING: T2T dump is empty. Only UID will be emulated.\r\n");
        } else {
            platformLog("[CE] T2T dump ready: %u pages\r\n", t2_pages);
        }
    }
    state = IDLE;
    platformLog("ListenIni() OK, state=%d, uid=%s\r\n", state, hex2Str(discParam.lmConfigPA.nfcid, (discParam.lmConfigPA.nfcidLen == RFAL_LM_NFCID_LEN_07) ? 7 : 4));
    return true;
}


/****************************************************************************************/
/*Emunlate Cycle*/
/****************************************************************************************/
/*============================================================================*/
/**
 * @brief ListenerCycle - Card Emulation main cycle
 *
 * Non-blocking function called periodically from FreeRTOS task.
 * Uses discParam initialized by ListenerIni() to enter Listen mode.
 * After REQA / ANTICOLL / SELECT, when RFAL_NFC_STATE_ACTIVATED is reached,
 * card type is visible on the reader (UID / ATQA / SAK).
 * At this stage, memory emulation is not performed,
 * and T2T / MFC commands are minimally responded to or ignored.
 * 
 * @retval None
 */
/*============================================================================*/
void ListenerCycle(void)
{
    ReturnCode   err;
    rfalNfcState nfcState;

    /* Periodic RFAL processing */
    rfalNfcWorker();
    nfcState = rfalNfcGetState();

    /* If CE stop request received from external (optional) */
    if (s_listener_stop) {
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        state     = IDLE;
        s_cePhase = CE_PHASE_IDLE;
        s_listener_stop = false;
        platformLog("[CE] listener stop -> IDLE\r\n");
        return;
    }

    switch (state)
    {
    /* ListenerIni() not yet called */
    case NOTINIT:
        return;

    /* First entry after ListenIni()
       → Set to START_DISCOVERY once to start Discover */
    case IDLE:
        state = START_DISCOVERY;
        break;

    /* Start Discover: Enter Listen mode and wait for REQA/ANTICOLL/SELECT */
    case START_DISCOVERY:
        platformLog("[CE] IDLE -> DISCOVERY persona=%d\r\n", g_persona);

        /* Clean up if previous session remains */
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);

        err = rfalNfcDiscover(&discParam);
        if (err != RFAL_ERR_NONE) {
            platformLog("[CE] rfalNfcDiscover() error=%d\r\n", err);
            state     = IDLE;
            s_cePhase = CE_PHASE_IDLE;
            break;
        }

        state     = DISCOVERY;
        s_cePhase = CE_PHASE_IDLE;
        break;

    /* Waiting for field detection / ACTIVATE */
    case DISCOVERY:
        switch (nfcState) {
        case RFAL_NFC_STATE_IDLE:
        case RFAL_NFC_STATE_LISTEN_TECHDETECT:
        case RFAL_NFC_STATE_LISTEN_ACTIVATION:
            /* Reader is still approaching, wait here without doing anything */
            break;

        case RFAL_NFC_STATE_ACTIVATED:
        {
            rfalNfcDevice *dev = NULL;

            /* Get active device (CE target) */
            if (rfalNfcGetActiveDevice(&dev) == RFAL_ERR_NONE && dev != NULL) {
                s_ceActiveDev = dev;

                /* Correct activeDev fields based on discParam
                   (UID, ATQA, SAK, ATS, etc.) */
                CeApplyDiscParamToActiveDev(dev);
            }

            /* For T4T: Check status with rfalNfcDataExchangeGetStatus() then arm RX */
            if( Emu_GetPersona() == EMU_PERSONA_T4T )
            {
                err = rfalNfcDataExchangeGetStatus();
                platformLog("RFAL_NFC_STATE_ACTIVATED (T4T) err = %d\r\n", err);
                
                if (err == RFAL_ERR_BUSY) break;
                if (err != RFAL_ERR_NONE) { break; }

                /* Arm RX so first frame can be received in ISO-DEP CE as well */
                s_ceRxData   = NULL;
                s_ceRxRcvLen = 0;

                err = CeArmRx();
                if (err != RFAL_ERR_NONE) {
                    platformLog("[CE] DataExchangeStart(RX) error=%d\r\n", err);
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                    break;
                }

                state     = DATAEXCHANGE;
                s_cePhase = CE_PHASE_WAIT_RX;

                platformLog("[CE] DISCOVERY -> DATAEXCHANGE <T4T>\r\n");
                break;
            }

            /* For T2T: Call rfalNfcDataExchangeStart() first to check already received data */
            /* T2T command received in LISTEN_ACTIVATED of rfal_nfc.c is stored in gNfcDev.rxBuf.rfBuf */
            s_ceRxData   = NULL;
            s_ceRxRcvLen = NULL;
            
            /* Get pointer to already received data (returns gNfcDev.rxBuf.rfBuf when called in ACTIVATED state) */
            err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
            if (err != RFAL_ERR_NONE) {
                platformLog("[CE] DataExchangeStart error=%d\r\n", err);
                state     = START_DISCOVERY;
                s_cePhase = CE_PHASE_IDLE;
                break;
            }
            
            /* Check if data was already received */
            err = rfalNfcDataExchangeGetStatus();
            if (err == RFAL_ERR_BUSY) {
                /* Still in data exchange, process in next loop */
                state     = DATAEXCHANGE;
                s_cePhase = CE_PHASE_WAIT_RX;
                break;
            }
            
            if (err == RFAL_ERR_NONE && s_ceRxData != NULL && s_ceRxRcvLen != NULL && (*s_ceRxRcvLen > 0U)) {
                /* First frame already received (T2T command received in LISTEN_ACTIVATED of rfal_nfc.c) */
                state     = DATAEXCHANGE;
                s_cePhase = CE_PHASE_WAIT_RX;
                platformLog("[CE] DISCOVERY -> DATAEXCHANGE (first frame already received, len=%u bits)\r\n", *s_ceRxRcvLen);
                break; /* Process with CeHandleT2TCmdRx() in next loop */
            }
            
            /* If no data received, arm RX (normal case) */
            err = CeArmRx();
            if (err != RFAL_ERR_NONE) {
                platformLog("[CE] DataExchangeStart(RX) error=%d\r\n", err);
                state     = START_DISCOVERY;
                s_cePhase = CE_PHASE_IDLE;
                break;
            }
            
            state     = DATAEXCHANGE;
            s_cePhase = CE_PHASE_WAIT_RX;
            platformLog("[CE] DISCOVERY -> DATAEXCHANGE\r\n");
            break;
        }

        case RFAL_NFC_STATE_LISTEN_SLEEP:
            /* Reader sent DESELECT / HLTA → restart discovery */
            platformLog("[CE] LISTEN_SLEEP -> restart discovery\r\n");
            rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
            state     = START_DISCOVERY;
            s_cePhase = CE_PHASE_IDLE;
            break;

        default:
            break;
        }
        break;

    /* Data exchange stage with reader
       - At this stage, only UID-based type identification needs to be guaranteed,
         so T2T/MFC commands can be minimally responded to.
       - DESFire(T4T) is mostly handled by RFAL T4T CE */
    case DATAEXCHANGE:

        /* If field disappeared or session ended, return to DISCOVERY */
        if ((nfcState == RFAL_NFC_STATE_IDLE) || (nfcState == RFAL_NFC_STATE_LISTEN_SLEEP))
        {
            platformLog("[CE] field off -> restart discovery\r\n");
            rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
            state     = START_DISCOVERY;
            s_cePhase = CE_PHASE_IDLE;
            break;
        }

        switch (s_cePhase)
        {
            case CE_PHASE_IDLE:
            {
                /* Idle state: no action needed, wait for next discovery */
                break;
            }
            case CE_PHASE_WAIT_RX:
            {
                /* 1) Check RX completion/error status */
                err = rfalNfcDataExchangeGetStatus();
                if (err == RFAL_ERR_BUSY) {
                    break;
                }
                if (err != RFAL_ERR_NONE) {
                    platformLog("[CE] RX error=%d\r\n", err);
                    /* LINK_LOSS (37) means Poller turned off field, treat as normal termination */
                    if (err == RFAL_ERR_LINK_LOSS) {
                        platformLog("[CE] RX: link loss (poller field off) - normal termination\r\n");
                        state     = START_DISCOVERY;
                        s_cePhase = CE_PHASE_IDLE;
                        break;
                    }
                    /* For other errors, try to re-arm RX */
                    err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
                    if (err != RFAL_ERR_NONE) {
                        platformLog("[CE] Re-arm RX after error failed: %d\r\n", err);
                        state     = START_DISCOVERY;
                        s_cePhase = CE_PHASE_IDLE;
                    }
                    break;
                }

                /* 2) Validate RX data pointer/length */
                if (!s_ceRxData || !s_ceRxRcvLen || (*s_ceRxRcvLen == 0U)) {
                    /* If RX is empty, arm RX */
                    err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
                    if (err != RFAL_ERR_NONE) {
                        platformLog("[CE] Arm RX error=%d\r\n", err);
                        state     = START_DISCOVERY;
                        s_cePhase = CE_PHASE_IDLE;
                    }
                    break;
                }

                /* 3) Parse RX */
                uint16_t rxBits  = *s_ceRxRcvLen;
                uint16_t rxBytes = rfalConvBitsToBytes(rxBits);

                /* RX logging (enable only when needed) */
#if 0
                if (rxBytes >= 1) {
                    uint8_t cmd = s_ceRxData[0];
                    if (cmd == 0x30 && rxBytes >= 2) {
                        platformLog("[CE][RX] T2T READ: page=%u (cmd=0x%02X)\r\n", 
                                   (unsigned)s_ceRxData[1], cmd);
                    } else if (cmd == 0x60) {
                        platformLog("[CE][RX] T2T GET_VERSION (cmd=0x%02X)\r\n", cmd);
                    } else {
                        platformLog("[CE][RX] cmd=0x%02X len=%uB\r\n", cmd, (unsigned)rxBytes);
                    }
                }
#endif

                /* 4) If REQA/WUPA comes in DATAEXCHANGE, return to discovery */
                if ((s_ceRxData[0] == NFCA_CMD_REQA) || (s_ceRxData[0] == NFCA_CMD_WUPA)) {
                    platformLog("[CE] REQA/WUPA in DATAEXCHANGE -> restart discovery\r\n");
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                    break;
                }

                /* 5) Dispatch based on persona */
                g_ceTxLenBytes = 0U;
                g_ceTxPending  = false;

                if (g_persona == EMU_PERSONA_MFC) {
                    /* MIFARE Classic emulation */
                    if (CeHandleMfcCmdRx(s_ceRxData, rxBits) == true) {
                        break;
                    }
                } else {
                    /* T2T / RAW emulation */
                    if (CeHandleT2TCmdRx(s_ceRxData, rxBits) == true) {
                        break;
                    }
                }

                /* 6) If command was not handled, re-arm RX or return to discovery */
                platformLog("[CE] Non-T2T cmd=0x%02X (%uB) -> re-arm RX\r\n",
                            s_ceRxData[0], (unsigned)rxBytes);

                err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
                if (err != RFAL_ERR_NONE) {
                    platformLog("[CE] re-arm RX error=%d\r\n", err);
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                }
                break;
            }
            case CE_PHASE_WAIT_TX:
            {
                /* WAIT_TX state: not currently used in this implementation */
                /* If needed in future, handle TX completion here */
                break;
            }
            case CE_PHASE_DATAEX:
            {
                /* Wait for TX completion */
                err = rfalNfcDataExchangeGetStatus();
                if (err == RFAL_ERR_BUSY) {
                    break;
                }

                /* RFAL_ERR_LINK_LOSS means Poller turned off field, treat as normal termination */
                if (err == RFAL_ERR_LINK_LOSS) {
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                    break;
                }

                if (err != RFAL_ERR_NONE) {
                    platformLog("[CE] TX err=%d\r\n", err);
                    /* Some errors are retryable, so try to re-arm RX */
                    if (err == RFAL_ERR_TIMEOUT || err == RFAL_ERR_FRAMING || err == RFAL_ERR_CRC || err == RFAL_ERR_PAR) {
                        g_ceTxPending  = false;
                        g_ceTxLenBytes = 0U;
                        *s_ceRxRcvLen = 0;
                        
                        err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
                        if (err == RFAL_ERR_NONE) {
                            s_cePhase = CE_PHASE_WAIT_RX;
                            break;
                        }
                    }
                    /* For non-retryable errors or re-arm failure, return to discovery */
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                    break;
                }

                /* TX completed normally → Arm RX to receive next command */
                g_ceTxPending  = false;
                g_ceTxLenBytes = 0U;
                
                /* Reset only RX buffer pointer (fast processing) */
                *s_ceRxRcvLen = 0;

                /* Immediately arm RX after TX completion (no delay) */
                err = rfalNfcDataExchangeStart(NULL, 0, &s_ceRxData, &s_ceRxRcvLen, RFAL_FWT_NONE);
                if (err != RFAL_ERR_NONE) {
                    platformLog("[CE] Arm RX err=%d\r\n", err);
                    state     = START_DISCOVERY;
                    s_cePhase = CE_PHASE_IDLE;
                    break;
                }

                s_cePhase = CE_PHASE_WAIT_RX;
                break;
            }
        }
        break;//end of DATAEXCHANGE

    default:
        break;
    }
}


/*============================================================================*/
/**
 * @brief ListenerNotif - Listener(CE) dedicated notification callback
 * 
 * Handles RFAL NFC state change notifications for listener mode.
 * 
 * @param[in] st RFAL NFC state
 * @retval None
 */
/*============================================================================*/
static void ListenerNotif(rfalNfcState st)
{
    rfalNfcDevice *dev = NULL;

    switch (st)
    {
    case RFAL_NFC_STATE_START_DISCOVERY:
        /* If was previously active, handle session termination here */
        if (s_wasActivated) {
            platformLog("[ListenerNotif] Deactivated -> restart discovery\r\n");
            s_wasActivated = false;
            s_active_dev   = NULL;
            Listener_OnDeactivated();    /* Upper hook */

        } else {
            //platformLog("[CE] Start discovery\r\n");
        }
        /* Multi-select flag has no meaning in listener but initialize existing variable */
        multiSel = false;
        break;

    case RFAL_NFC_STATE_ACTIVATED:
        platformLog("[ListenerNotif] ListenerNotif: ACTIVATED\r\n");
        if (rfalNfcGetActiveDevice(&dev) == RFAL_ERR_NONE && dev) {
            s_active_dev   = dev;
            s_wasActivated = true;

        } else {
            platformLog("[ListenerNotif] Activated, but no active device\r\n");
        }
        break;

    case RFAL_NFC_STATE_DATAEXCHANGE:
        platformLog("[ListenerNotif] Data exchange\r\n");
        break;

    case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
        //platformLog("[CE] Data exchange done\r\n");
        break;

    case RFAL_NFC_STATE_LISTEN_SLEEP:
        platformLog("[ListenerNotif] Listen sleep\r\n");
        break;

    /* Poller-only states are ignored in listener */
    case RFAL_NFC_STATE_POLL_TECHDETECT:
    case RFAL_NFC_STATE_POLL_SELECT:
        break;

    case RFAL_NFC_STATE_WAKEUP_MODE:
        platformLog("[ListenerNotif] Wake-up mode\r\n");
        break;

    default:
        platformLog("[ListenerNotif] State=%d\r\n", st);
        break;
    }
}


/*============================================================================*/
/**
 * @brief ceT2T_FromDump - Type 2 / NTAG dump-based CE response
 * 
 * Builds CE response based on T2T dump data for various T2T commands.
 * 
 * @param[in] rx Received data buffer
 * @param[in] rxLenB Received data length in bytes
 * @param[out] tx Transmission buffer
 * @param[in] txSize Transmission buffer size
 * @retval Response length in bytes, or 0 on error
 */
/*============================================================================*/
static uint16_t ceT2T_FromDump(const uint8_t *rx, uint16_t rxLenB, uint8_t *tx, uint16_t txSize)
{
    uint8_t cmd  = rx[0];
    uint8_t addr = rx[1];

    platformLog("[CE] T2T cmd=0x%02X len=%uB\r\n", cmd, (unsigned)rxLenB);

    if (!rx || rxLenB < 2 || !tx || txSize < 4) {
        return 0;
    }

    const uint16_t page_cnt = nfc_ctx_get_t2t_page_count();
    if (page_cnt == 0) {
        return 0;
    }

    /* Clamp address to actual page range */
    uint16_t page = addr;
    if (page >= page_cnt) {
        page = page_cnt - 1;
    }

    switch (cmd)
    {
        case 0x30:
        {
            platformLog("[CE] 0x30 T2T READ startPage=%u\r\n", rx[1]);
            if (rxLenB < 2U) return 0;

            uint8_t  startPage   = rx[1];
            uint8_t  pagesToRead = 4U;  
            uint8_t  page_buf[4];
            uint16_t totalBytes  = 0;

            for (uint8_t i = 0; i < pagesToRead; i++) {
                uint16_t pageIndex = (uint16_t)startPage + i;

                if (pageIndex >= page_cnt || !nfc_ctx_get_t2t_page(pageIndex, page_buf)) {
                    /* Pad with 0 if out of range */
                    memset(page_buf, 0x00, sizeof(page_buf));
                }

                if (totalBytes + 4U > txSize) break;
                memcpy(&tx[totalBytes], page_buf, 4U);
                totalBytes += 4U;
            }

            return totalBytes;
        }

        /* 0x3A: FAST_READ multiple pages (N * 4 bytes) */
        case 0x3A:
        {
            platformLog("[CE] T2T FAST_READ start=%u end=%u\r\n", rx[1], rx[2]);
            if (rxLenB < 3) {
                return 0;
            }

            uint8_t end_addr = rx[2];
            uint16_t start   = page;
            uint16_t end     = end_addr;

            if (end >= page_cnt) end = page_cnt - 1;
            if (end < start)     end = start;

            uint16_t num_pages = (end - start + 1);
            uint16_t total_len = num_pages * 4U;

            if (txSize < total_len) {
                total_len = (txSize / 4U) * 4U;
                num_pages = total_len / 4U;
            }

            uint8_t page_buf[4];
            uint16_t out_off = 0;

            for (uint16_t p = 0; p < num_pages; p++) {
                uint16_t cur_page = start + p;
                if (!nfc_ctx_get_t2t_page(cur_page, page_buf)) {
                    memset(page_buf, 0x00, sizeof(page_buf));
                }
                memcpy(&tx[out_off], page_buf, 4);
                out_off += 4;
            }

            return out_off;
        }

        /* 0x60: GET_VERSION (NTAG / Ultralight-C series) */
        case 0x60:
        {
            uint8_t ver[8];
            if (!nfc_ctx_get_t2t_version(ver)) {
                /* NAK if no version info */
                return 0;
            }

            if (txSize < sizeof(ver)) {
                return 0;
            }

            memcpy(tx, ver, sizeof(ver));
            return (uint16_t)sizeof(ver);
        }

        /* 0xA2: WRITE one page (4 bytes) → Update dump so it's reflected when read again later */
        case 0xA2:
        {
            if (rxLenB < 6) {
                return 0;
            }

            uint8_t data[4];
            memcpy(data, &rx[2], 4);

            nfc_ctx_set_t2t_page(page, data);

            /* NTAG usually returns ACK one byte (0x0A / 0x0F / 0x00, etc.) */
            if (txSize < 1) return 0;
            tx[0] = 0x0A;   // generic ACK
            return 1;
        }

        default:
            break;
    }

    /* Unsupported command */
    return 0;
}

