/* See COPYING.txt for license details. */

#include <stdbool.h>


/* NFC-A short commands */
#define NFCA_CMD_REQA   0x26  /* 7-bit */
#define NFCA_CMD_WUPA   0x52  /* 7-bit */

/* Type 2 Tag commands */
#define T2T_CMD_READ        0x30
#define T2T_CMD_FAST_READ   0x3A
#define T2T_CMD_WRITE       0xA2  /* 4 bytes */
#define T2T_CMD_SECTOR_SEL  0xC2
#define T2T_CMD_GET_VERSION 0x60
#define T2T_CMD_COMPAT_WRITE 0xA0 /* optional / legacy */


/* MIFARE Classic commands (shared with T2T GET_VERSION 0x60 — persona disambiguates) */
#define MFC_CMD_AUTH_A      0x60
#define MFC_CMD_AUTH_B      0x61
#define MFC_CMD_READ        0x30
#define MFC_CMD_WRITE       0xA0
#define MFC_CMD_HALT        0x50

typedef enum {
    EMU_PERSONA_T4T = 0,
    EMU_PERSONA_T2T,
    EMU_PERSONA_MFC,    /* MIFARE Classic emulation (Crypto-1) */
    EMU_PERSONA_RAW,    /* Use ATQA/SAK from read card as-is */
} EmuPersona_t;

/**
 * @brief Emu_SetPersona - Set emulation persona
 * 
 * @param[in] p Persona type
 * @retval None
 */
void Emu_SetPersona(EmuPersona_t p);

/**
 * @brief Emu_GetPersona - Get emulation persona
 * 
 * @retval Current persona type
 */
EmuPersona_t Emu_GetPersona(void);

/**
 * @brief ListenIni - Initialize NFC listener
 * 
 * @retval true Initialization successful
 * @retval false Initialization failed
 */
bool ListenIni(void);

/**
 * @brief ListenerCycle - Main listener processing loop
 * 
 * @retval None
 */
void ListenerCycle(void);

/**
 * @brief ListenerRequestStop - Request listener stop
 * 
 * @retval None
 */
void ListenerRequestStop(void);

/**
 * @brief ListenerGetLastRx - Get last received data
 * 
 * @param[out] lenBits Pointer to store received length in bits
 * @retval Pointer to last received data buffer, or NULL if no data
 */
const uint8_t* ListenerGetLastRx(uint16_t *lenBits);
