/* See COPYING.txt for license details. */

/*
 ******************************************************************************
 * nfc_poller.c - NFC Poller Mode (Reader/Card Reading)
 ******************************************************************************
 * 
 * [Purpose]
 * - Implements NFC-A reader functionality
 * - Detects and reads various NFC card types (T2T, MIFARE Classic, T4T)
 * - Performs card type identification and data reading
 * 
 * [State Machine]
 * NOTINIT → START_DISCOVERY → DISCOVERY
 * 
 * [Key Flow]
 * 1. ReadIni(): Initialize RFAL in poller mode
 *    - Configure discovery parameters (POLL_TECH_A)
 *    - Start rfalNfcDiscover()
 * 
 * 2. ReadCycle(): Main processing loop
 *    - START_DISCOVERY: Start discovery process
 *    - DISCOVERY: Poll for cards, handle activation
 * 
 * 3. Card Detection and Type Identification:
 *    - ISO14443A/NFC-A card detected
 *    - Analyze ATQA + SAK to determine card type:
 *      * MIFARE Classic (SAK=0x08/0x18): Use key dictionary attack
 *      * Type 2 Tag (T2T): Read NTAG/Ultralight
 *      * Type 4 Tag (T4T): ISO-DEP APDU exchange
 * 
 * 4. T2T Reading (m1_t2t_read_ntag()):
 *    - GET_VERSION (0x60): Get NTAG version info
 *    - READ (0x30): Read pages sequentially (0~135)
 *    - FAST_READ (0x3A): Read multiple pages at once
 *    - Save data to nfc_ctx and SD card (.nfc file)
 * 
 * [Card Type Detection]
 * - MIFARE Classic: ATQA=0x0400/0x4400, SAK=0x08 (1K) or 0x18 (4K)
 * - Type 2 Tag: ATQA=0x0044, SAK=0x00
 * - Type 4 Tag: ISO-DEP protocol
 * 
 ******************************************************************************
 */
#include "nfc_poller.h"
#include "utils.h"
#include "rfal_nfc.h"
#include "rfal_t2t.h"
#include "rfal_isoDep.h"
#include "rfal_nfcv.h"
#include "rfal_st25tb.h"

#include "st25r3916.h"
#include "st25r3916_com.h"
#include "rfal_utils.h"

#include "rfal_analogConfig.h"
#include "rfal_rf.h"
#include "uiView.h"
#include "nfc_driver.h"
#include "nfc_poller.h"
#include "common/nfc_ctx.h"
#include "common/nfc_storage.h"
#include "m1_sdcard.h"
#include "m1_storage.h"
#include "common/nfc_fileio.h"
#include "logger.h"
#include "mfc_crypto1.h"
#include "picopass/picopass_poller.h"
#include <stdio.h>

#define NOTINIT              0     /*!< Demo State:  Not initialized        */
#define START_DISCOVERY      1     /*!< Demo State:  Start Discovery        */
#define DISCOVERY            2     /*!< Demo State:  Discovery              */

/*
 ******************************************************************************
 * MIFARE CLASSIC Helper Functions
 ******************************************************************************
 */

#define MFC_DICT_PATH   "NFC/system/mf_classic_dict.nfc"

/* Streaming key dictionary: reads one key at a time from SD card to avoid
 * allocating thousands of keys on the stack. */
typedef struct {
    nfcfio_t io;
    char     line[32];
    bool     is_open;
    uint16_t count;   /* Total keys counted (for logging) */
} mfc_key_iter_t;

static bool mfc_key_iter_open(mfc_key_iter_t *it);
static bool mfc_key_iter_next(mfc_key_iter_t *it, uint8_t key_out[MFC_KEY_LEN]);
static void mfc_key_iter_rewind(mfc_key_iter_t *it);
static void mfc_key_iter_close(mfc_key_iter_t *it);

static void mfc_get_layout_from_sak(uint8_t sak, uint16_t *outSectors, uint16_t *outBlocks);
static uint16_t mfc_sector_to_first_block(uint16_t sector);

/* Returns true if SAK indicates any MIFARE Classic variant */
static bool mfc_is_classic_sak(uint8_t sak)
{
    switch (sak) {
    case 0x01: /* Classic 1K (TNP3xxx) */
    case 0x08: /* Classic 1K */
    case 0x09: /* MIFARE Mini */
    case 0x10: /* Plus 2K SL2 */
    case 0x11: /* Plus 4K SL2 */
    case 0x18: /* Classic 4K */
    case 0x19: /* Classic 2K */
    case 0x28: /* Classic EV1 1K */
    case 0x38: /* Classic EV1 4K */
        return true;
    default:
        return false;
    }
}

/* Forward declarations for NFC-V / ST25TB reading */
static void m1_nfcv_read(const rfalNfcDevice *dev);
static void m1_st25tb_read(const rfalNfcDevice *dev);
static void m1_read_mifareclassic(const rfalNfcDevice *dev);

extern uint8_t g_nfc_dump_buf[NFC_DUMP_BUF_SIZE];
extern uint8_t g_nfc_valid_bits[NFC_VALID_BITS_SIZE];

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

static rfalNfcDiscoverParam discParam;
static uint8_t              state = NOTINIT;
static bool                 multiSel;


/* NFC-A CE config */
/* 4-byte UIDs with first byte 0x08 would need random number for the subsequent 3 bytes.
 * 4-byte UIDs with first byte 0x*F are Fixed number, not unique, use for this demo
 * 7-byte UIDs need a manufacturer ID and need to assure uniqueness of the rest.*/

/*------------------------------------------------------------------------------------------*/
static void PollerNotif( rfalNfcState st );
static void m1_t2t_read_ntag(const rfalNfcDevice *dev);
static ReturnCode GetVersion_Ntag(uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rcvLen);
static uint16_t LogParsedNtagVersion(const uint8_t *version, uint16_t len);
static ReturnCode PwdAuth_Ntag(const uint8_t pwd[4], uint8_t pack[2]);
static bool ntag_generate_amiibo_pwd(const uint8_t *uid, uint8_t uid_len, uint8_t pwd_out[4], uint8_t pack_out[2]);

/*------------------------------------------------------------------------------------------*/
#define SET_FAMILY(fmt, ...)  do { snprintf(NFC_Family, sizeof(NFC_Family), fmt, ##__VA_ARGS__); } while(0)

/*
 * DESFire detection via ISO-DEP APDU: send GET_VERSION (0x60) wrapped
 * in ISO 7816 framing. If the card responds with 0x91AF it's DESFire.
 */
static bool desfire_probe_iso_dep(const rfalNfcDevice* dev)
{
    if (!dev || dev->rfInterface != RFAL_NFC_INTERFACE_ISODEP)
        return false;

    static rfalIsoDepApduBufFormat txApdu;
    static rfalIsoDepApduBufFormat rxApdu;
    static rfalIsoDepBufFormat     tmpBuf;
    uint16_t rxLen = 0;

    /* DESFire GET_VERSION wrapped APDU: CLA=0x90, INS=0x60, P1=0, P2=0, Lc=0 */
    txApdu.apdu[0] = 0x90;
    txApdu.apdu[1] = 0x60;
    txApdu.apdu[2] = 0x00;
    txApdu.apdu[3] = 0x00;
    txApdu.apdu[4] = 0x00; /* Le */

    const rfalIsoDepInfo *info = &dev->proto.isoDep.info;

    rfalIsoDepApduTxRxParam param;
    param.txBuf    = &txApdu;
    param.txBufLen = 5;
    param.rxBuf    = &rxApdu;
    param.rxLen    = &rxLen;
    param.tmpBuf   = &tmpBuf;
    param.FWT      = info->FWT;
    param.dFWT     = info->dFWT;
    param.FSx      = info->FSx;
    param.ourFSx   = RFAL_ISODEP_DEFAULT_FSC;
    param.DID      = info->supDID ? info->DID : RFAL_ISODEP_NO_DID;

    ReturnCode err = rfalIsoDepStartApduTransceive(param);
    if (err != RFAL_ERR_NONE)
        return false;

    /* Poll for completion (max ~100 iterations) */
    for (int i = 0; i < 100; i++) {
        rfalNfcWorker();
        err = rfalIsoDepGetApduTransceiveStatus();
        if (err == RFAL_ERR_NONE) break;
        if (err != RFAL_ERR_BUSY) return false;
    }
    if (err != RFAL_ERR_NONE)
        return false;

    /* Check response: DESFire answers with SW1=0x91, SW2=0xAF (more data)
     * or 0x91 0x00 (success). Response has 7+ bytes of version data + SW. */
    if (rxLen >= 2) {
        uint8_t sw1 = rxApdu.apdu[rxLen - 2];
        uint8_t sw2 = rxApdu.apdu[rxLen - 1];
        if (sw1 == 0x91 && (sw2 == 0xAF || sw2 == 0x00)) {
            return true;
        }
    }

    return false;
}

/* Magic card (Gen1A) detection: Send backdoor command 0x40 then 0x43 */
static bool mfclassic_is_magic_backdoor_supported(void)
{
    /* Gen1A detection requires sending raw short frames (0x40, 0x43) outside
     * of normal ISO14443A framing. The ST25R3916 can do 7-bit frames via
     * RFAL but it requires the card to be in HALT state first. For now,
     * this is best-effort — we attempt it but don't block on failure. */
    uint8_t txBuf[1];
    uint8_t rxBuf[4];
    uint16_t rcvLen = 0;

    /* Send HALT first */
    txBuf[0] = 0x50;
    rfalTransceiveBlockingTxRx(txBuf, 1, rxBuf, sizeof(rxBuf),
                               &rcvLen, RFAL_TXRX_FLAGS_DEFAULT,
                               rfalConvMsTo1fc(5));

    /* Send magic backdoor wakeup: 0x40 as 7-bit short frame */
    txBuf[0] = 0x40;
    rcvLen = 0;
    ReturnCode err = rfalTransceiveBlockingTxRx(
        txBuf, 1, rxBuf, sizeof(rxBuf), &rcvLen,
        (uint32_t)(RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP),
        rfalConvMsTo1fc(5));

    if (err != RFAL_ERR_NONE || rcvLen < 1)
        return false;

    /* If we got an ACK (0x0A, 4-bit), send 0x43 */
    if ((rxBuf[0] & 0x0F) == 0x0A) {
        txBuf[0] = 0x43;
        rcvLen = 0;
        err = rfalTransceiveBlockingTxRx(
            txBuf, 1, rxBuf, sizeof(rxBuf), &rcvLen,
            (uint32_t)(RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP),
            rfalConvMsTo1fc(5));
        if (err == RFAL_ERR_NONE && rcvLen >= 1 && (rxBuf[0] & 0x0F) == 0x0A) {
            return true;
        }
    }

    return false;
}
/*============================================================================*/
/**
 * @brief nfc_a_fill_uid_and_family - Fill NFC-A UID and Family (SAK/ATQA priority → type as auxiliary)
 * 
 * Fills UID and Family information for NFC-A devices based on SAK/ATQA values,
 * with device type as auxiliary information.
 * 
 * @param[in] dev Pointer to rfalNfcDevice (type=RFAL_NFC_LISTEN_TYPE_NFCA)
 * @retval None
 */
/*============================================================================*/
static void nfc_a_fill_uid_and_family(const rfalNfcDevice *dev)
{
    const rfalNfcaListenDevice* a = &dev->dev.nfca;

    /* Fill UID */
    NFC_ID_LEN = dev->nfcidLen;
    memset(NFC_ID, 0, sizeof(NFC_ID));
    ST_MEMCPY(NFC_ID, dev->nfcid, NFC_ID_LEN);
    strcpy(NFC_UID, hex2Str(dev->nfcid, dev->nfcidLen));

    /* Extract ATQA/SAK */
    const uint8_t sak   = a->selRes.sak;
    const uint8_t atqa0 = a->sensRes.anticollisionInfo;  /* ATQA LSB */
    const uint8_t atqa1 = a->sensRes.platformInfo;       /* ATQA MSB */
    const rfalNfcaListenDeviceType t = a->type;

    NFC_ATQA[0] = (char)atqa0;
    NFC_ATQA[1] = (char)atqa1;
    NFC_SAK = (char)sak;

    platformLog("[NFCA] type=%d ATQA=%02X%02X SAK=%02X\r\n", t, atqa0, atqa1, sak);

    /* 1) MIFARE Classic / Plus variants — classify by SAK */
    if (mfc_is_classic_sak(sak)) {
        const char *variant = "MIFARE Classic";
        switch (sak) {
        case 0x01: variant = "MIFARE Classic 1K (TNP3xxx)"; break;
        case 0x08: variant = "MIFARE Classic 1K"; break;
        case 0x09: variant = "MIFARE Mini 0.3K"; break;
        case 0x10: variant = "MIFARE Plus 2K (SL2)"; break;
        case 0x11: variant = "MIFARE Plus 4K (SL2)"; break;
        case 0x18: variant = "MIFARE Classic 4K"; break;
        case 0x19: variant = "MIFARE Classic 2K"; break;
        case 0x28: variant = "MIFARE Classic EV1 1K"; break;
        case 0x38: variant = "MIFARE Classic EV1 4K"; break;
        default:   variant = "MIFARE Classic"; break;
        }
        SET_FAMILY("%s", variant);
        return;
    }

    /* 2) Type 4A / DESFire — SAK bit 5 set (0x20) */
    if (sak == 0x20 || t == RFAL_NFCA_T4T || t == RFAL_NFCA_T4T_NFCDEP) {
        if (desfire_probe_iso_dep(dev)) SET_FAMILY("MIFARE DESFire (Type 4A)");
        else                            SET_FAMILY("Type 4A (ISO-DEP)");
        return;
    }

    /* 3) Ultralight/NTAG — SAK=0x00 with T2T type, or ATQA indicating Ultralight */
    if (sak == 0x00 && (t == RFAL_NFCA_T2T || atqa0 == 0x44)) {
        SET_FAMILY("Ultralight/NTAG");
        return;
    }

    /* 4) Topaz (Type 1 Tag, rare) */
    if (t == RFAL_NFCA_T1T) {
        SET_FAMILY("Topaz (Type 1)");
        return;
    }

    /* 5) Others */
    SET_FAMILY("NFC-A (unspecified)");
}


/*============================================================================*/
/**
 * @brief ReadCycle - Main NFC poller processing loop
 * 
 * Main processing loop for NFC poller mode. Handles state machine transitions
 * and card detection/reading operations. Processes discovered cards and
 * identifies their types (MIFARE Classic, T2T, T4T, etc.).
 * 
 * State machine:
 * - START_DISCOVERY: Starts discovery process
 * - DISCOVERY: Polls for cards and handles activation
 * 
 * @retval None
 */
/*============================================================================*/
/*------------------------------------------------------------------------------------------*/
void ReadCycle(void)
{
    static rfalNfcDevice *nfcDevice;

    rfalNfcWorker();                                    /* Run RFAL worker periodically */

    switch (state)
    {
/********************************** State : DISCOVERY *************************************/
    case START_DISCOVERY:
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        rfalNfcDiscover(&discParam);

        multiSel = false;
        state    = DISCOVERY;
        //platformLog("Poller START_DISCOVERY\r\n");
        break;  //↓ 
/************************************** State : DISCOVERY *****************************************/
    case DISCOVERY:
        //rfalNfcDevice *list = NULL; uint8_t cnt = 0;
        //rfalNfcGetDevicesFound(&list, &cnt);
        //if (cnt > 0) platformLog("Found cnt=%u\r\n", cnt);
        if (rfalNfcIsDevActivated(rfalNfcGetState()))
        {
            //platformLog("Poller rfalNfcGetState\r\n");
        
            m1_wdt_reset();

            bool notifyRead = false;

            rfalNfcGetActiveDevice(&nfcDevice); /* Get active device */

            uint8_t ctx_err = FillNfcContextFromDevice(nfcDevice);
            if (ctx_err != 0) {
                platformLog("FillNfcContextFromDevice err=%d (type=%d)\r\n",
                            (int)ctx_err, nfcDevice ? (int)nfcDevice->type : -1);
                /* If needed, can retry by setting state=START_DISCOVERY here */
            }else platformLog("Fill NFC Context Successed\r\n");

            switch (nfcDevice->type)
            {
                case RFAL_NFC_LISTEN_TYPE_NFCA:   /* ISO14443A / NFC-A card (MFC, UL/NTAG, DESFire, Magic) */
                {
                    isNFCCardFound = true;
                    nfc_tx_type    = NFC_TX_A;

                    // For logging: use dev->nfcid
                    platformLog("ISO14443A/NFC-A TAG found. type=0x%02X, UID=%s\r\n", nfcDevice->dev.nfca.type, hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                    strcpy(NFC_Type, "ISO14443A/NFC-A");

                    // ATQA/SAK (access via RFAL structure fields)
                    const uint8_t sak   = nfcDevice->dev.nfca.selRes.sak;
                    const uint8_t atqa0 = nfcDevice->dev.nfca.sensRes.anticollisionInfo;
                    const uint8_t atqa1 = nfcDevice->dev.nfca.sensRes.platformInfo;

                    /* --- MIFARE Classic identification by SAK --- */
                    bool isMfcClassic = mfc_is_classic_sak(sak);

                    if (isMfcClassic) {
                        platformLog("MIFARE Classic detected. ATQA=%02X%02X, SAK=%02X, UIDLen=%u\r\n",
                                    atqa0, atqa1, sak, nfcDevice->nfcidLen);

                        /* Attempt to read sectors using key dictionary */
                        m1_read_mifareclassic(nfcDevice);
                    }

                    /* Store in emulation context (UID/ATQA/SAK) */
                    Emu_SetNfcA(nfcDevice->nfcid, nfcDevice->nfcidLen, atqa0, atqa1, sak);
                    //platformLog("Emu_SetNfcA nfcDevice->nfcidLen=%u\r\n", nfcDevice->nfcidLen);

                    // === Auto-set Family/UID here ===
                    nfc_a_fill_uid_and_family(nfcDevice);

                    /* Additional reading for T1/T2/T4/DEP only if not MIFARE Classic */
                    if (!isMfcClassic) {
                        if (nfcDevice->dev.nfca.type == RFAL_NFCA_T1T)
                            platformLog("NFC Type 1 Tag Read More\r\n");
                        else if (nfcDevice->dev.nfca.type == RFAL_NFCA_T2T) {
                            platformLog("NFC Type 2 Tag Read More\r\n");
                            m1_t2t_read_ntag(nfcDevice); // Improve to t2t
                        }
                        else if (nfcDevice->dev.nfca.type == RFAL_NFCA_T4T)
                            platformLog("NFC Type 4 Tag Read More\r\n");
                        else if (nfcDevice->dev.nfca.type == RFAL_NFCA_NFCDEP)
                            platformLog("NFC DEP Read More\r\n");
                        else if (nfcDevice->dev.nfca.type == RFAL_NFCA_T4T_NFCDEP)
                            platformLog("NFC Type 4 Tag DEP Read More\r\n");
                    }


                    notifyRead = true;
                }

                break;
                /*******************************************************************************/
                case RFAL_NFC_LISTEN_TYPE_NFCB:   /* ISO14443B / NFC-B */
                    platformLog("ISO14443B/NFC-B TAG found. UID=%s\r\n",
                                hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                    strcpy(NFC_Type, "ISO14443B/NFC-B");
                    nfc_tx_type = NFC_TX_B;
                    notifyRead  = true;
                    break;

                /*******************************************************************************/
                case RFAL_NFC_LISTEN_TYPE_NFCF:   /* FeliCa / NFC-F */
                    isNFCCardFound = true;
                    platformLog("FeliCa/NFC-F TAG found. NFCID=%s\r\n",
                                hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                    strcpy(NFC_Type, "Felica/NFC-F");
                    nfc_tx_type = NFC_TX_F;
                    notifyRead  = true;
                    break;

                /*******************************************************************************/
                case RFAL_NFC_LISTEN_TYPE_NFCV:   /* ISO15693 / NFC-V — includes PicoPass/iCLASS */
                {
                    uint8_t devUID[RFAL_NFCV_UID_LEN];
                    ST_MEMCPY(devUID, nfcDevice->nfcid, nfcDevice->nfcidLen);
                    REVERSE_BYTES(devUID, RFAL_NFCV_UID_LEN);  /* Reverse for display */
                    platformLog("ISO15693/NFC-V TAG found. UID=%s\r\n",
                                hex2Str(devUID, RFAL_NFCV_UID_LEN));
                    nfc_tx_type = NFC_TX_V;

                    /* Try PicoPass / iCLASS first — it rides on ISO15693 PHY */
                    {
                        static PicopassData pp_data;
                        PicopassReadResult pp_res = picopass_poller_read(nfcDevice, &pp_data);
                        if (pp_res == PICOPASS_READ_OK || pp_res == PICOPASS_READ_ERR_AUTH) {
                            /* It IS a PicoPass card */
                            strcpy(NFC_Type, "PicoPass/iCLASS");
                            SET_FAMILY("PicoPass");
                            isNFCCardFound = true;

                            /* Fill context for save/display */
                            nfc_run_ctx_t *c = nfc_ctx_get();
                            c->head.tech     = M1NFC_TECH_V;
                            c->head.family   = M1NFC_FAM_ICLASS;
                            c->head.uid_len  = 8;
                            memcpy(c->head.uid, pp_data.csn, 8);

                            /* Copy block data into dump buffer */
                            memset(g_nfc_dump_buf, 0, NFC_DUMP_BUF_SIZE);
                            memset(g_nfc_valid_bits, 0, NFC_VALID_BITS_SIZE);
                            uint8_t num_blocks = pp_data.block_count;
                            if (num_blocks > PICOPASS_MAX_APP_LIMIT)
                                num_blocks = PICOPASS_MAX_APP_LIMIT;
                            for (uint8_t i = 0; i < num_blocks; i++) {
                                memcpy(&g_nfc_dump_buf[i * 8], pp_data.blocks[i].data, 8);
                                g_nfc_valid_bits[i >> 3] |= (uint8_t)(1u << (i & 7));
                            }
                            nfc_ctx_set_dump(8, num_blocks, 0,
                                             g_nfc_dump_buf, g_nfc_valid_bits,
                                             (num_blocks > 0) ? num_blocks - 1 : 0, true);

                            if (pp_res == PICOPASS_READ_ERR_AUTH) {
                                platformLog("[PP] Card detected but auth failed — partial read\r\n");
                            }
                            notifyRead = true;
                            break;
                        }
                    }

                    /* Not PicoPass — fall through to standard NFC-V read */
                    strcpy(NFC_Type, "ISO15693/NFC-V");
                    SET_FAMILY("ISO15693");

                    /* Read NFC-V blocks */
                    m1_nfcv_read(nfcDevice);

                    notifyRead  = true;
                }
                break;

                /*******************************************************************************/
                case RFAL_NFC_LISTEN_TYPE_ST25TB:
                {
                    platformLog("ST25TB TAG found. UID=%s\r\n",
                                hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                    strcpy(NFC_Type, "ST25TB");
                    SET_FAMILY("ST25TB");

                    /* Read ST25TB blocks */
                    m1_st25tb_read(nfcDevice);

                    notifyRead = true;
                }
                break;

                /*******************************************************************************/
                /* CE/P2P types are ignored in READ-ONLY mode */
                case RFAL_NFC_LISTEN_TYPE_AP2P:
                case RFAL_NFC_POLL_TYPE_AP2P:
                case RFAL_NFC_POLL_TYPE_NFCA:
                case RFAL_NFC_POLL_TYPE_NFCF:
                    platformLog("Non-reader mode type detected (ignored in READ-ONLY): type=%d\r\n",
                                nfcDevice->type);
                    notifyRead = false;
                    break;

                /*******************************************************************************/
                default:
                    platformLog("Unknown type=%d\r\n", nfcDevice->type);
                    notifyRead = false;
                    break;
            } /* switch(nfcDevice->type) */

            /* Common: Update UID buffer and length */
            strcpy(NFC_UID, hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
            NFC_ID_LEN = nfcDevice->nfcidLen;

            for (int i = 0; i < 20; ++i) {
                NFC_ID[i] = 0;
            }
            for (int i = 0; i < nfcDevice->nfcidLen; ++i) {
                NFC_ID[i] = nfcDevice->nfcid[i];
            }

            /* Notify read completion */
            if (notifyRead) {
                m1_wdt_reset();

                m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
            }

            /* Always deactivate to idle and return to re-discovery loop */
            rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);

            state = START_DISCOVERY;
        }
        else
        {
            /* Nothing found. Continue polling in next cycle */
            //platformLog("rfalNfcIsDevActivated false\r\n\r\n");
        }

        m1_wdt_reset();

        break; /* DEMO_ST_DISCOVERY */

/************************************** State : NOTINIT *****************************************/
    case NOTINIT:
        //platformLog("NOTINIT\r\n");
        /* fallthrough */
    default:
        //platformLog("default\r\n");
        break;
    } /* switch(state) */
}


/*============================================================================*/
/**
 * @brief ReadIni - Initialize NFC poller (READ-ONLY mode)
 *
 * Poller-only initialization. CE/P2P disabled.
 * Validates RFAL/DiscParam settings and enters START_DISCOVERY state.
 *
 * @retval true  Initialization successful
 * @retval false Initialization failed
 */
/*============================================================================*/
bool ReadIni(void)
{
    ReturnCode err = RFAL_ERR_NONE;

    /* 1) RFAL Initialize (retry 2 times) */
    for (int i = 0; i < 2; i++) {
        err = rfalNfcInitialize();
        //platformLog("rfalNfcInitialize() = %d\r\n", err);
        if (err == RFAL_ERR_NONE) break;
        vTaskDelay(5);
    }
    if (err != RFAL_ERR_NONE) return false;

    /* 2) Discovery parameters: Explicitly set for Poller-only mode */
    rfalNfcDefaultDiscParams(&discParam);

    discParam.devLimit       = 1U;
    discParam.totalDuration  = 1000U;                      /* Discovery window (adjust if needed) */
    discParam.notifyCb       = PollerNotif;                  /* Keep if in use */
#if defined(RFAL_COMPLIANCE_MODE_NFC)
    discParam.compMode       = RFAL_COMPLIANCE_MODE_NFC;   /* Recommended for application */
#endif

    /* CE/P2P disabled: Enable only Poller in techs2Find */
    discParam.techs2Find     = RFAL_NFC_TECH_NONE;

#if RFAL_FEATURE_NFCA
    discParam.techs2Find    |= RFAL_NFC_POLL_TECH_A;
#endif
#if RFAL_FEATURE_NFCB
    discParam.techs2Find    |= RFAL_NFC_POLL_TECH_B;
#endif
#if RFAL_FEATURE_NFCF
    discParam.techs2Find    |= RFAL_NFC_POLL_TECH_F;
#endif
#if RFAL_FEATURE_NFCV
    discParam.techs2Find    |= RFAL_NFC_POLL_TECH_V;
#endif
#if RFAL_FEATURE_ST25TB
    discParam.techs2Find    |= RFAL_NFC_POLL_TECH_ST25TB;
#endif

    /* Never enabled (READ ONLY) */
    /* discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;        */
    /* discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;      */
    /* discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A/F;       */

#if ST25R95
    discParam.isoDepFS       = RFAL_ISODEP_FSXI_128; /* ST25R95 Limit */
#endif

    /* 3) Verify configuration is valid by calling Discover once, then deactivate to Idle */
    err = rfalNfcDiscover(&discParam);
    rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
    if (err != RFAL_ERR_NONE) {
        platformLog("rfalNfcDiscover() check failed: %d\r\n", err);
        return false;
    }

    /* 4) Enter state machine starting point */
    state = START_DISCOVERY;

    //platformLog("ReadIni() OK, state=%d\r\n", state);
    return true;
}


/*============================================================================*/
/**
 * @brief PollerNotif - Poller notification callback
 * 
 * Handles RFAL NFC state change notifications for poller mode.
 * Manages wake-up mode, device detection, multiple device selection,
 * and discovery state transitions.
 * 
 * @param[in] st RFAL NFC state
 * @retval None
 */
/*============================================================================*/
static void PollerNotif( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;

    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        platformLog("Wake Up mode started \r\n");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        if( discParam.wakeupEnabled )
        {
            platformLog("Wake Up mode terminated. Polling for devices \r\n");
        }
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Check if in case of multiple devices, selection is already attempted */
        if( (!multiSel) )
        {
            multiSel = true;
            /* Multiple devices were found, activate first of them */
            rfalNfcGetDevicesFound( &dev, &devCnt );
            rfalNfcSelect( 0 );

            platformLog("Multiple Tags detected: %d \r\n", devCnt);
        }
        else
        {
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
        }
    }
    else if( st == RFAL_NFC_STATE_START_DISCOVERY )
    {
        /* Clear mutiple device selection flag */
        multiSel = false;
    }
}


/*============================================================================*/
/**
 * @brief GetVersion_Ntag - Send GET_VERSION command to NTAG
 * 
 * Sends NTAG GET_VERSION command (0x60) and receives 8-byte version response.
 * This command retrieves version information including vendor, product,
 * size, and protocol information from NTAG/Ultralight-C tags.
 * 
 * @param[out] rxBuf Buffer to store received version data
 * @param[in] rxBufLen Size of receive buffer (must be at least 8)
 * @param[out] rcvLen Pointer to store actual received length
 * @retval RFAL_ERR_NONE Success
 * @retval RFAL_ERR_PARAM Invalid parameters
 * @retval Other RFAL error codes
 */
/*============================================================================*/
static ReturnCode GetVersion_Ntag(uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rcvLen)
{
    uint8_t cmd = 0x60; /* NTAG GET_VERSION command*/

    if ((rxBuf == NULL) || (rcvLen == NULL) || (rxBufLen < 8U)) {
        return RFAL_ERR_PARAM;
    }

    *rcvLen = 0;

    return rfalTransceiveBlockingTxRx(&cmd, 1U, rxBuf, rxBufLen, rcvLen, RFAL_TXRX_FLAGS_DEFAULT, rfalConvMsTo1fc(5U));
}

/*============================================================================*/
/**
 * @brief PwdAuth_Ntag - Send PWD_AUTH (0x1B) command to NTAG/Ultralight
 *
 * Sends a 4-byte password to authenticate with a password-protected NTAG.
 * On success, the tag returns a 2-byte PACK (Password ACKnowledge).
 * After successful auth, protected pages become readable.
 *
 * @param[in]  pwd   4-byte password to send
 * @param[out] pack  2-byte PACK response from tag (valid on success)
 * @retval RFAL_ERR_NONE  Authentication successful
 * @retval Other          RFAL error (NAK = wrong password, timeout, etc.)
 */
/*============================================================================*/
static ReturnCode PwdAuth_Ntag(const uint8_t pwd[4], uint8_t pack[2])
{
    uint8_t  txBuf[5];
    uint8_t  rxBuf[2];
    uint16_t rcvLen = 0;

    txBuf[0] = 0x1B;  /* PWD_AUTH command */
    txBuf[1] = pwd[0];
    txBuf[2] = pwd[1];
    txBuf[3] = pwd[2];
    txBuf[4] = pwd[3];

    ReturnCode err = rfalTransceiveBlockingTxRx(
        txBuf, sizeof(txBuf),
        rxBuf, sizeof(rxBuf),
        &rcvLen,
        RFAL_TXRX_FLAGS_DEFAULT,
        rfalConvMsTo1fc(20U));

    if (err == RFAL_ERR_NONE && rcvLen >= 2 && pack != NULL) {
        pack[0] = rxBuf[0];
        pack[1] = rxBuf[1];
    }

    return err;
}

/*============================================================================*/
/**
 * @brief ntag_generate_amiibo_pwd - Generate Amiibo PWD_AUTH password from UID
 *
 * Derives the 4-byte password and 2-byte PACK for Amiibo (NTAG215) tags
 * using the well-known XOR algorithm. Requires a 7-byte UID.
 *
 * Algorithm:
 *   PWD[0] = 0xAA ^ UID[1] ^ UID[3]
 *   PWD[1] = 0x55 ^ UID[2] ^ UID[4]
 *   PWD[2] = 0xAA ^ UID[3] ^ UID[5]
 *   PWD[3] = 0x55 ^ UID[4] ^ UID[6]
 *   PACK   = 0x80, 0x80
 *
 * @param[in]  uid       7-byte UID from NTAG215
 * @param[in]  uid_len   UID length (must be 7)
 * @param[out] pwd_out   4-byte generated password
 * @param[out] pack_out  2-byte expected PACK
 * @retval true   Success
 * @retval false  Invalid UID length
 */
/*============================================================================*/
static bool ntag_generate_amiibo_pwd(const uint8_t *uid, uint8_t uid_len,
                                     uint8_t pwd_out[4], uint8_t pack_out[2])
{
    if (!uid || uid_len != 7 || !pwd_out || !pack_out)
        return false;

    pwd_out[0] = 0xAA ^ uid[1] ^ uid[3];
    pwd_out[1] = 0x55 ^ uid[2] ^ uid[4];
    pwd_out[2] = 0xAA ^ uid[3] ^ uid[5];
    pwd_out[3] = 0x55 ^ uid[4] ^ uid[6];

    pack_out[0] = 0x80;
    pack_out[1] = 0x80;

    return true;
}


/*============================================================================*/
/**
 * @brief LogParsedNtagVersion - Parse and log NTAG version information
 * 
 * Parses NTAG GET_VERSION response (8 bytes) and logs detailed version information
 * including vendor, product, sub-type, version number, size, and protocol.
 * Determines total page count based on size code:
 * - 0x0F: NTAG213 (45 pages, 144B user)
 * - 0x11: NTAG215 (135 pages, 504B user)
 * - 0x13: NTAG216 (231 pages, 888B user)
 * 
 * @param[in] version Pointer to 8-byte version data from GET_VERSION response
 * @param[in] len Length of version data (should be 8)
 * @retval Total page count (0 if unknown or invalid)
 */
/*============================================================================*/
static uint16_t LogParsedNtagVersion(const uint8_t *version, uint16_t len)
{
	//Use attributes for the compressor warning issue
    uint16_t totalPages = 0;

    if ((version == NULL) || (len < 8U)) {
        platformLog("NTAG GET_VERSION parse skipped (len=%u)\r\n", len);
        return totalPages;
    }

    const uint8_t header  __attribute__((unused)) = version[0];
    const uint8_t vendor  = version[1];
    const uint8_t prod    = version[2];
    const uint8_t sub     = version[3];
    const uint8_t major   __attribute__((unused)) = version[4];
    const uint8_t minor   __attribute__((unused)) = version[5];
    const uint8_t size    = version[6];
    const uint8_t proto   = version[7];

    const char *vendorStr __attribute__((unused)) = (vendor == 0x04) ? "NXP" : "Unknown";
    const char *prodStr   __attribute__((unused)) = (prod == 0x04) ? "NTAG" : "Unknown";
    const char *subStr    __attribute__((unused)) = (sub == 0x02) ? "Standard" : "Unknown";

    const char *sizeStr __attribute__((unused)) = "Unknown";
    if (size == 0x0F) {
        sizeStr    = "NTAG213 [144B user]";
        totalPages = 45U;
    }
    else if (size == 0x11) {
        sizeStr    = "NTAG215 [504B user]";
        totalPages = 135U;
    }
    else if (size == 0x13) {
        sizeStr    = "NTAG216 [888B user]";
        totalPages = 231U;
    }

    const char *protoStr __attribute__((unused)) = (proto == 0x03) ? "ISO14443-3" : "Unknown";

    platformLog("NTAG VERSION parsed: hdr=0x%02X vendor=%s(0x%02X) prod=%s(0x%02X)\r\n", header, vendorStr, vendor, prodStr, prod);
    platformLog("sub=%s(0x%02X) ver=%u.%u size=%s(0x%02X) proto=%s(0x%02X)\r\n", subStr, sub, (unsigned)major, (unsigned)minor, sizeStr, size, protoStr, proto);

    if (totalPages == 0U) {
        platformLog("NTAG VERSION: unknown size code 0x%02X, using default page count\r\n", size);
    } else {
        platformLog("NTAG VERSION: detected %u total pages\r\n", totalPages);
    }

    return totalPages;
}

/*============================================================================*/
/**
 * @brief m1_t2t_read_ntag - Read Type 2 Tag (NTAG/Ultralight) memory
 * 
 * Reads Type 2 Tag memory pages using T2T READ commands.
 * Performs GET_VERSION to determine page count, then reads all pages
 * sequentially. Parses NDEF TLV if present.
 * 
 * @param[in] dev Pointer to NFC device (unused, kept for compatibility)
 * @retval None
 */
/*============================================================================*/
static void m1_t2t_read_ntag(const rfalNfcDevice *dev)
{
    (void)dev; // Temporarily unused

    uint8_t  buf[16];               // T2T READ response buffer
    uint8_t* dump     = g_nfc_dump_buf;
    uint16_t dumpSize = NFC_DUMP_BUF_SIZE;
    uint16_t offset   = 0;
    uint16_t rcvLen   = 0;
    uint16_t max_page = 45U;        // Default to NTAG213 span when size is unknown
    ReturnCode err;

    // Clear previous NDEF / dump (optional)
    nfc_ctx_clear_t2t_ndef();
    nfc_ctx_clear_dump();

    uint8_t version[8] = {0};
    bool    ver_ok     = false;

    for (uint8_t attempt = 0; attempt < 2 && !ver_ok; attempt++) {
        if (attempt > 0) {
            osDelay(10);  // Wait before retry
        }
        
        memset(version, 0x00, sizeof(version));
        rcvLen = 0;
        err = GetVersion_Ntag(version, sizeof(version), &rcvLen);

        /* If all response bytes are 0x00, consider it a failure */
        bool all_zero = true;
        for (uint8_t i = 0; i < sizeof(version); i++) {
            if (version[i] != 0x00) { all_zero = false; break; }
        }

        if ((err == RFAL_ERR_NONE) && (rcvLen == sizeof(version)) && !all_zero) {
            ver_ok = true;
        }
    }

    if (ver_ok) {
        max_page = LogParsedNtagVersion(version, rcvLen);
        nfc_ctx_set_t2t_version(version, (uint8_t)rcvLen);
        platformLog("GET_VER OK: %u pages\r\n", max_page);
    } else {
        /* If GET_VERSION fails, get page count from context */
        uint16_t ctx_pages = nfc_ctx_get_t2t_page_count();
        if (ctx_pages > 0) {
            max_page = ctx_pages;
            platformLog("GET_VER fail, using ctx: %u pages\r\n", max_page);
        } else {
            platformLog("GET_VER fail, using default: %u pages\r\n", max_page);
        }
    }



    /* --- PWD_AUTH: Unlock password-protected pages --- */
    /* Priority: 1) Manual password (user-entered)
     *           2) Captured password (sniffed from reader during emulation)
     *           3) Auto-generated (Amiibo XOR derivation for NTAG215) */
    {
        uint8_t pwd[4], pack_rx[2] = {0, 0};
        nfc_pwd_source_t pwd_src = NFC_PWD_SRC_NONE;
        bool auth_attempted = false;

        /* Try manual or captured password first */
        if (nfc_ctx_get_best_pwd(pwd, &pwd_src)) {
            const char *src_str = (pwd_src == NFC_PWD_SRC_MANUAL) ? "manual" : "captured";
            platformLog("[T2T] PWD_AUTH (%s): pwd=%s\r\n", src_str, hex2Str(pwd, 4));
            ReturnCode auth_err = PwdAuth_Ntag(pwd, pack_rx);
            if (auth_err == RFAL_ERR_NONE) {
                platformLog("[T2T] PWD_AUTH OK (%s): PACK=%02X %02X\r\n",
                            src_str, pack_rx[0], pack_rx[1]);
            } else {
                platformLog("[T2T] PWD_AUTH failed (%s, err=%d)\r\n", src_str, auth_err);
            }
            auth_attempted = true;
            /* Clear manual password after use (one-shot) */
            if (pwd_src == NFC_PWD_SRC_MANUAL)
                nfc_ctx_clear_manual_pwd();
        }

        /* If no manual/captured password, try Amiibo auto-auth for NTAG215 */
        if (!auth_attempted && max_page == 135U && ver_ok && version[6] == 0x11) {
            nfc_run_ctx_t* ctx = nfc_ctx_get();
            if (ctx && ctx->head.uid_len == 7) {
                uint8_t pack[2];
                if (ntag_generate_amiibo_pwd(ctx->head.uid, ctx->head.uid_len, pwd, pack)) {
                    platformLog("[T2T] Amiibo PWD_AUTH: pwd=%s\r\n", hex2Str(pwd, 4));
                    ReturnCode auth_err = PwdAuth_Ntag(pwd, pack_rx);
                    if (auth_err == RFAL_ERR_NONE) {
                        platformLog("[T2T] PWD_AUTH OK: PACK=%02X %02X\r\n", pack_rx[0], pack_rx[1]);
                    } else {
                        platformLog("[T2T] PWD_AUTH failed (err=%d) — non-Amiibo or locked tag\r\n", auth_err);
                    }
                }
            }
        }
    }

    // Read blocks 0 ~ N (T2T READ reads 4 blocks at a time, so increment by 4)
    for (uint8_t blk = 0; blk < max_page; blk += 4) {  // Read 4 blocks at a time
        rcvLen = 0;
        bool read_ok = false;
        
        /* Retry each READ command (max 3 times, to handle timing issues) */
        for (uint8_t retry = 0; retry < 3 && !read_ok; retry++) {
            /* Wait before retry (allow time for CE to respond) */
            if (retry > 0) {
                osDelay(10);  // Increase retry wait time (10ms)
            }
            
            /* Initialize buffer (prevent previous data residue) */
            memset(buf, 0x00, sizeof(buf));
            rcvLen = 0;
            
            err = rfalT2TPollerRead(blk, buf, sizeof(buf), &rcvLen);
            
            if (err == RFAL_ERR_NONE && rcvLen >= 16) {  // T2T READ returns 16 bytes (4 blocks)
                read_ok = true;
                /* Log only critical blocks */
                if (blk == 0 || blk == 4) {
                    platformLog("READ blk%u: %s\r\n", blk, hex2Str(buf, 8));
                }
            } else {
                /* Log only critical blocks on failure */
                if ((blk == 0 || blk == 4) && retry == 0) {
                    platformLog("READ blk%u fail: err=%d rcv=%u\r\n", blk, err, rcvLen);
                }
                /* If field loss, don't retry anymore */
                if (err == RFAL_ERR_LINK_LOSS) {
                    break;
                }
            }
        }
        
        /* Calculate number of pages to save */
        uint8_t pages_to_save = 4;
        if (blk + pages_to_save > max_page) {
            pages_to_save = max_page - blk;
        }
        
        if (!read_ok) {
            /* Stop if field loss */
            if (err == RFAL_ERR_LINK_LOSS) {
                break;
            }
            /* Fill failed blocks with 0 at corresponding position (prevent data shift) */
            for (uint8_t i = 0; i < pages_to_save; i++) {
                uint16_t page_idx = blk + i;
                if (page_idx >= max_page) break;
                
                uint16_t save_offset = page_idx * 4;
                if (save_offset + 4 <= dumpSize) {
                    memset(&dump[save_offset], 0x00, 4);
                }
            }
        } else {
            /* On success: Store received data at corresponding block position */
            for (uint8_t i = 0; i < pages_to_save; i++) {
                uint16_t page_idx = blk + i;
                if (page_idx >= max_page) break;
                
                uint16_t save_offset = page_idx * 4;
                if (save_offset + 4 <= dumpSize) {
                    /* Copy page data from buf */
                    uint8_t *src = &buf[i * 4];
                    uint8_t *dst = &dump[save_offset];
                    memcpy(dst, src, 4);
                }
            }
        }
        
        /* Update offset to next block position (regardless of success/failure) */
        offset = (blk + 4) * 4;
        
        /* Log progress (every 16 blocks) */
        if ((blk + 4) % 16 == 0 || (blk + 4) >= max_page) {
            platformLog("T2T read progress: %u/%u pages\r\n", (unsigned)((blk + 4 > max_page) ? max_page : blk + 4), (unsigned)max_page);
        }
    }

    osDelay(5);

    // Actual dump length and page count
    uint16_t dump_len  = offset;
    uint16_t num_pages = dump_len / 4;

    if (num_pages == 0) {
        platformLog("T2T: no pages dumped\r\n");
        return;
    }

    // Mark all pages as valid in valid_bits for now
    memset(g_nfc_valid_bits, 0xFF, (num_pages + 7) / 8);

    // Update context dump metadata
    nfc_ctx_set_dump(4,                // unit_size: 4 bytes per page
                     num_pages,        // unit_count
                     0,                // origin
                     g_nfc_dump_buf,   // data pointer
                     g_nfc_valid_bits, // valid bits
                     num_pages - 1,    // max_seen_unit
                     true);            // has_dump = true

    // ---- TLV parsing (find NDEF) ----
    const uint8_t firstDataPage = 4;
    uint8_t *p   = dump + (firstDataPage * 4);
    uint8_t *end = dump + dump_len;

    if (p >= end) {
        platformLog("T2T: dump too short (offset=%u)\r\n", dump_len);
        return;
    }

    while (p < end) {
        uint8_t t = *p++;

        if (t == 0x00) {
            // NULL TLV
            continue;
        }
        if (t == 0xFE) {
            // Terminator TLV
            platformLog("T2T: Terminator TLV reached, no more TLVs\r\n");
            break;
        }

        if (p >= end) break;

        uint16_t len = *p++;  // Simple 1-byte length handling

        if (t == 0x03) {      // NDEF TLV
            if (p + len > end) {
                len = (uint16_t)(end - p);  // Defensive
            }

            platformLog("T2T NDEF TLV found, len=%u\r\n", len);
            nfc_ctx_set_t2t_ndef(p, len);
#if 0
            //nfc_ctx_dump_t2t_ndef();   // Debug hexdump + ASCII
            //nfc_ctx_dump_t2t_pages();
#endif
            return;
        } else {
            // Skip uninterested TLV by len
            if (p + len > end) break;
            p += len;
        }
    }

    platformLog("T2T: NDEF TLV not found (scan from page=%u)\r\n", firstDataPage);
}




/*
 ******************************************************************************
 * MIFARE CLASSIC helper functions
 ******************************************************************************
*/

/*============================================================================*/
/* Streaming key dictionary iterator — reads one key at a time from SD card   */
/*============================================================================*/

static int mfc_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static bool mfc_key_iter_open(mfc_key_iter_t *it)
{
    if (!it) return false;
    it->count   = 0;
    it->is_open = false;

    if (!nfcfio_open_read(&it->io, MFC_DICT_PATH)) {
        platformLog("[MFC] dict open failed: %s\r\n", MFC_DICT_PATH);
        return false;
    }
    it->is_open = true;
    return true;
}

static bool mfc_key_iter_next(mfc_key_iter_t *it, uint8_t key_out[MFC_KEY_LEN])
{
    if (!it || !it->is_open) return false;

    int n;
    while ((n = nfcfio_getline(&it->io, it->line, sizeof(it->line))) >= 0) {
        char *p = it->line;

        /* Skip whitespace */
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') continue;
        if (*p == '#')  continue;

        /* Parse 12 hex chars → 6 bytes */
        uint8_t key[MFC_KEY_LEN];
        bool ok = true;
        for (int i = 0; i < MFC_KEY_LEN && ok; i++) {
            int hi = mfc_hex_nibble(*p++);
            int lo = mfc_hex_nibble(*p++);
            if (hi < 0 || lo < 0) { ok = false; break; }
            key[i] = (uint8_t)((hi << 4) | lo);
        }
        if (!ok) continue;

        memcpy(key_out, key, MFC_KEY_LEN);
        it->count++;
        return true;
    }
    return false;
}

static void mfc_key_iter_rewind(mfc_key_iter_t *it)
{
    if (!it || !it->is_open) return;
    nfcfio_close(&it->io);
    it->is_open = false;
    it->count   = 0;
    if (nfcfio_open_read(&it->io, MFC_DICT_PATH)) {
        it->is_open = true;
    }
}

static void mfc_key_iter_close(mfc_key_iter_t *it)
{
    if (!it) return;
    if (it->is_open) {
        nfcfio_close(&it->io);
        it->is_open = false;
    }
}



/*============================================================================*/
/* MIFARE Classic read using software Crypto-1 + streaming key dictionary     */
/*============================================================================*/

static void mfc_get_layout_from_sak(uint8_t sak, uint16_t *outSectors, uint16_t *outBlocks)
{
    switch (sak) {
    case 0x09: /* Mini */
        *outSectors = 5;
        *outBlocks  = 20;
        break;
    case 0x01: /* Classic 1K TNP3xxx */
    case 0x08: /* Classic 1K */
    case 0x28: /* Classic EV1 1K */
        *outSectors = 16;
        *outBlocks  = 64;
        break;
    case 0x10: /* Plus 2K SL2 */
    case 0x19: /* Classic 2K */
        *outSectors = 32;
        *outBlocks  = 128;
        break;
    case 0x11: /* Plus 4K SL2 */
    case 0x18: /* Classic 4K */
    case 0x38: /* Classic EV1 4K */
        *outSectors = 40;
        *outBlocks  = 256;
        break;
    default:
        platformLog("[MFC] unknown SAK 0x%02X, fallback to 1K layout\r\n", sak);
        *outSectors = 16;
        *outBlocks  = 64;
        break;
    }
}

static uint16_t mfc_sector_to_first_block(uint16_t sector)
{
    if (sector < 32)
        return (uint16_t)(sector * 4);
    else
        return (uint16_t)(128 + (sector - 32) * 16);
}

static void m1_read_mifareclassic(const rfalNfcDevice *dev)
{
    const rfalNfcaListenDevice *nfca = &dev->dev.nfca;
    uint16_t totalSectors = 0;
    uint16_t totalBlocks  = 0;
    uint16_t maxBlocks    = NFC_DUMP_MAX_UNITS;

    mfc_get_layout_from_sak(nfca->selRes.sak, &totalSectors, &totalBlocks);
    if (totalBlocks > maxBlocks) totalBlocks = maxBlocks;

    /* UID: first 4 bytes for Crypto-1 (even for 7-byte UID cards) */
    uint8_t uid4[4];
    if (dev->nfcidLen >= 7) {
        /* For 7-byte UID: use bytes 3..6 (the second cascade level) */
        memcpy(uid4, &dev->nfcid[3], 4);
    } else {
        memcpy(uid4, dev->nfcid, 4);
    }

    memset(g_nfc_dump_buf, 0x00, NFC_DUMP_BUF_SIZE);
    memset(g_nfc_valid_bits, 0x00, NFC_VALID_BITS_SIZE);

    platformLog("[MFC] start dump: sectors=%u blocks=%u\r\n", totalSectors, totalBlocks);

    uint16_t lastSeenBlock  = 0;
    uint16_t successSectors = 0;

    mfc_key_iter_t iter;
    if (!mfc_key_iter_open(&iter)) {
        platformLog("[MFC] no key dict, skip MFC dump\r\n");
        return;
    }

    crypto1_state_t cstate;

    for (uint16_t sector = 0; sector < totalSectors; sector++) {
        uint16_t firstBlock     = mfc_sector_to_first_block(sector);
        uint16_t blocksInSector = (sector < 32 || totalSectors <= 16) ? 4 : 16;
        if (firstBlock >= totalBlocks) break;

        bool sectorAuthed = false;
        uint8_t key[MFC_KEY_LEN];

        /* Rewind key iterator for each sector */
        mfc_key_iter_rewind(&iter);

        while (mfc_key_iter_next(&iter, key) && !sectorAuthed) {
            m1_wdt_reset();

            /* Try Key A */
            if (mfc_auth(&cstate, uid4, (uint8_t)firstBlock, MFC_AUTH_CMD_A, key)) {
                sectorAuthed = true;
                platformLog("[MFC] sector %u auth OK KeyA\r\n", sector);
                break;
            }

            /* Try Key B */
            if (mfc_auth(&cstate, uid4, (uint8_t)firstBlock, MFC_AUTH_CMD_B, key)) {
                sectorAuthed = true;
                platformLog("[MFC] sector %u auth OK KeyB\r\n", sector);
                break;
            }
        }

        if (!sectorAuthed) {
            platformLog("[MFC] sector %u auth FAILED\r\n", sector);
            continue;
        }

        successSectors++;

        /* Read all blocks of authenticated sector */
        for (uint16_t bi = 0; bi < blocksInSector; bi++) {
            uint16_t blockNo = firstBlock + bi;
            if (blockNo >= totalBlocks) break;

            uint8_t *dst = &g_nfc_dump_buf[blockNo * MFC_BLOCK_SIZE];

            if (mfc_read_block_crypto(&cstate, (uint8_t)blockNo, dst)) {
                g_nfc_valid_bits[blockNo >> 3] |= (uint8_t)(1u << (blockNo & 0x7));
                if (blockNo + 1 > lastSeenBlock) lastSeenBlock = blockNo + 1;
            } else {
                platformLog("[MFC] read block %u failed\r\n", blockNo);
            }
        }
    }

    mfc_key_iter_close(&iter);

    nfc_ctx_set_dump(MFC_BLOCK_SIZE, totalBlocks, 0,
                     g_nfc_dump_buf, g_nfc_valid_bits,
                     lastSeenBlock, (lastSeenBlock > 0));

    platformLog("[MFC] dump done: successSectors=%u lastBlock=%u\r\n",
                successSectors, lastSeenBlock);
}

/*============================================================================*/
/* NFC-V (ISO15693) block reading                                             */
/*============================================================================*/

static void m1_nfcv_read(const rfalNfcDevice *dev)
{
    uint8_t rxBuf[32];
    uint16_t rcvLen = 0;

    /* Get system information to determine block count and size */
    ReturnCode err = rfalNfcvPollerGetSystemInformation(
        RFAL_NFCV_REQ_FLAG_DEFAULT, dev->nfcid, rxBuf, sizeof(rxBuf), &rcvLen);

    uint16_t blockCount = 0;
    uint8_t  blockSize  = 4; /* default */

    if (err == RFAL_ERR_NONE && rcvLen >= 12) {
        /* System info response: flags(1) + DSFID + UID(8) + ... */
        /* Per ISO15693: if INFO_FLAGS bit 0 set, includes block info */
        uint8_t infoFlags = rxBuf[1]; /* After response flags byte */
        /* Parse based on standard system info layout */
        int offset = 10; /* skip flags(1) + infoFlags(1) + UID(8) */
        if (infoFlags & 0x01) { /* DSFID present */
            offset++;
        }
        if (infoFlags & 0x02) { /* AFI present */
            offset++;
        }
        if (infoFlags & 0x04) { /* Memory size present */
            if (offset + 2 <= (int)rcvLen) {
                blockCount = (uint16_t)(rxBuf[offset] + 1); /* Number of blocks - 1 */
                blockSize  = (uint8_t)(rxBuf[offset + 1] + 1); /* Block size - 1 */
            }
        }
        platformLog("[NFC-V] SysInfo: blocks=%u blockSize=%u\r\n", blockCount, blockSize);
    } else {
        /* Fallback: try reading blocks until failure */
        blockCount = 64; /* Common default */
        platformLog("[NFC-V] SysInfo failed, trying %u blocks\r\n", blockCount);
    }

    if (blockCount == 0) return;
    if (blockSize > 32) blockSize = 32;

    memset(g_nfc_dump_buf, 0x00, NFC_DUMP_BUF_SIZE);
    memset(g_nfc_valid_bits, 0x00, NFC_VALID_BITS_SIZE);
    uint32_t maxSeen = 0;

    for (uint16_t blk = 0; blk < blockCount; blk++) {
        rcvLen = 0;
        err = rfalNfcvPollerReadSingleBlock(
            RFAL_NFCV_REQ_FLAG_DEFAULT, dev->nfcid, (uint8_t)blk,
            rxBuf, sizeof(rxBuf), &rcvLen);

        if (err == RFAL_ERR_NONE && rcvLen >= (uint16_t)(1 + blockSize)) {
            /* First byte is response flags, then data */
            uint32_t offset_dst = (uint32_t)blk * blockSize;
            if (offset_dst + blockSize <= NFC_DUMP_BUF_SIZE) {
                memcpy(&g_nfc_dump_buf[offset_dst], &rxBuf[1], blockSize);
                g_nfc_valid_bits[blk >> 3] |= (uint8_t)(1u << (blk & 7));
                if (blk > maxSeen) maxSeen = blk;
            }
        } else {
            /* Stop on first read error (likely end of memory) */
            if (blk > 0) break;
        }
        m1_wdt_reset();
    }

    if (maxSeen > 0) {
        nfc_ctx_set_dump(blockSize, blockCount, 0,
                         g_nfc_dump_buf, g_nfc_valid_bits,
                         maxSeen, true);
        platformLog("[NFC-V] dump done: %u blocks\r\n", maxSeen + 1);
    }
}

/*============================================================================*/
/* ST25TB block reading                                                       */
/*============================================================================*/

static void m1_st25tb_read(const rfalNfcDevice *dev)
{
    (void)dev;
    rfalSt25tbBlock blockData;
    uint8_t blockSize = RFAL_ST25TB_BLOCK_LEN; /* 4 bytes */
    uint16_t blockCount = 64; /* ST25TB512 has 16 blocks, ST25TB2K has 64 */

    memset(g_nfc_dump_buf, 0x00, NFC_DUMP_BUF_SIZE);
    memset(g_nfc_valid_bits, 0x00, NFC_VALID_BITS_SIZE);
    uint32_t maxSeen = 0;

    for (uint16_t blk = 0; blk < blockCount; blk++) {
        ReturnCode err = rfalSt25tbPollerReadBlock((uint8_t)blk, &blockData);

        if (err == RFAL_ERR_NONE) {
            uint32_t offset = (uint32_t)blk * blockSize;
            if (offset + blockSize <= NFC_DUMP_BUF_SIZE) {
                memcpy(&g_nfc_dump_buf[offset], blockData, blockSize);
                g_nfc_valid_bits[blk >> 3] |= (uint8_t)(1u << (blk & 7));
                if (blk > maxSeen) maxSeen = blk;
            }
        } else {
            if (blk > 0) break; /* End of memory */
        }
        m1_wdt_reset();
    }

    if (maxSeen > 0) {
        nfc_ctx_set_dump(blockSize, maxSeen + 1, 0,
                         g_nfc_dump_buf, g_nfc_valid_bits,
                         maxSeen, true);
        platformLog("[ST25TB] dump done: %u blocks\r\n", maxSeen + 1);
    }
}


/*============================================================================*/
/* Extern wrapper functions — expose static helpers for use by m1_nfc.c       */
/*============================================================================*/

bool nfc_poller_is_classic_sak(uint8_t sak)
{
    return mfc_is_classic_sak(sak);
}

void nfc_poller_read_mfc(const rfalNfcDevice *dev)
{
    m1_read_mifareclassic(dev);
}

void nfc_poller_read_t2t(const rfalNfcDevice *dev)
{
    m1_t2t_read_ntag(dev);
}

ReturnCode nfc_poller_pwd_auth(const uint8_t pwd[4], uint8_t pack[2])
{
    return PwdAuth_Ntag(pwd, pack);
}

ReturnCode nfc_poller_get_version(uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rcvLen)
{
    return GetVersion_Ntag(rxBuf, rxBufLen, rcvLen);
}
