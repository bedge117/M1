/* See COPYING.txt for license details. */

/*
*
*  m1_flipper_integration.c
*
*  Flipper Zero file import/replay integration for M1
*
*  Provides menu-callable functions to:
*  - Replay Flipper .sub files via the SI4463 radio
*  - Import Flipper .nfc files into M1 NFC saved format
*  - Import Flipper .rfid files into M1 RFID saved format
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_compile_cfg.h"

#ifdef M1_APP_FILE_IMPORT_ENABLE

#include "m1_flipper_integration.h"
#include "flipper_subghz.h"
#include "flipper_nfc.h"
#include "flipper_rfid.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "m1_ring_buffer.h"
#include "m1_nfc.h"
#include "m1_rfid.h"
#include "m1_lcd.h"
#include "m1_display.h"
#include "m1_file_browser.h"
#include "m1_file_util.h"
#include "m1_log_debug.h"
#include "ff.h"
#include "lfrfid.h"
#include "lfrfid_file.h"
#include "nfc_ctx.h"
#include "nfc_file.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG                "FlipperIntg"

#define FLIPPER_SUBGHZ_DIR          "0:/Flipper/SubGHz"
#define FLIPPER_NFC_DIR             "0:/Flipper/NFC"
#define FLIPPER_RFID_DIR            "0:/Flipper/RFID"
#define FLIPPER_DIR_ROOT            "0:/Flipper"

#define M1_NFC_SAVE_DIR             "0:/NFC"
#define M1_RFID_SAVE_DIR            "0:/RFID"

#define FLIPPER_PATH_MAX            256
#define FLIPPER_STATUS_LINE_Y       56

/************************** S T R U C T U R E S *******************************/

/***************************** V A R I A B L E S ******************************/

static char flipper_filepath[FLIPPER_PATH_MAX];

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void flipper_show_status(const char *line1, const char *line2);
static void flipper_show_result(const char *msg, bool success);
static bool flipper_wait_for_back(void);
static bool flipper_ensure_dir(const char *path);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
 * @brief  Show a two-line status message on screen
 */
/*============================================================================*/
static void flipper_show_status(const char *line1, const char *line2)
{
    u8g2_FirstPage(&m1_u8g2);
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        if (line1) u8g2_DrawStr(&m1_u8g2, 2, 12, line1);
        if (line2) u8g2_DrawStr(&m1_u8g2, 2, 28, line2);
    } while (u8g2_NextPage(&m1_u8g2));
}

/*============================================================================*/
/**
 * @brief  Show a result message (success/fail) and wait for BACK
 */
/*============================================================================*/
static void flipper_show_result(const char *msg, bool success)
{
    u8g2_FirstPage(&m1_u8g2);
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 12, success ? "Success" : "Failed");
        if (msg) u8g2_DrawStr(&m1_u8g2, 2, 28, msg);
        u8g2_DrawStr(&m1_u8g2, 2, FLIPPER_STATUS_LINE_Y, "Press BACK to exit");
    } while (u8g2_NextPage(&m1_u8g2));
}

/*============================================================================*/
/**
 * @brief  Block waiting for the BACK button press, return true on BACK
 */
/*============================================================================*/
static bool flipper_wait_for_back(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    while (1)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                xQueueReset(main_q_hdl);
                return true;
            }
        }
    }
}

/*============================================================================*/
/**
 * @brief  Ensure a directory exists on the SD card, create if needed
 */
/*============================================================================*/
static bool flipper_ensure_dir(const char *path)
{
    FILINFO fno;
    FRESULT fr;

    fr = f_stat(path, &fno);
    if (fr == FR_OK && (fno.fattrib & AM_DIR))
        return true;

    /* Try to create path components */
    char tmp[FLIPPER_PATH_MAX];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    for (char *p = tmp + 3; *p; p++) {  /* Skip "0:/" */
        if (*p == '/') {
            *p = '\0';
            f_mkdir(tmp);
            *p = '/';
        }
    }
    fr = f_mkdir(tmp);
    return (fr == FR_OK || fr == FR_EXIST);
}


/*============================================================================*/
/*                                                                            */
/*  SUB-GHZ: REPLAY FLIPPER .SUB FILES                                       */
/*                                                                            */
/*============================================================================*/

void sub_ghz_replay_flipper(void)
{
    DIR dir;
    FILINFO fno;
    FRESULT fr;
    bool found = false;
    uint8_t result;

    flipper_show_status("Import Sub-GHz", "Loading...");

    /* Ensure directory exists */
    flipper_ensure_dir(FLIPPER_SUBGHZ_DIR);

    /* Browse for .sub files */
    fr = f_opendir(&dir, FLIPPER_SUBGHZ_DIR);
    if (fr != FR_OK) {
        flipper_show_result("No Flipper/SubGHz dir", false);
        flipper_wait_for_back();
        return;
    }

    /* Find first .sub file */
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        size_t len = strlen(fno.fname);
        if (len > 4 && strcmp(&fno.fname[len - 4], ".sub") == 0) {
            snprintf(flipper_filepath, sizeof(flipper_filepath), "%s/%s",
                     FLIPPER_SUBGHZ_DIR, fno.fname);
            found = true;
            break;
        }
    }
    f_closedir(&dir);

    if (!found) {
        flipper_show_result("No .sub files found", false);
        flipper_wait_for_back();
        return;
    }

    M1_LOG_I(M1_LOGDB_TAG, "Sub-GHz Flipper replay: %s\r\n", flipper_filepath);
    flipper_show_status("Replaying:", fno.fname);

    /* Convert .sub → temp .sgh and enter native replay GUI.
     * This call blocks until the user presses BACK in the replay view. */
    result = sub_ghz_replay_flipper_file(flipper_filepath);
    if (result)
    {
        const char *err;
        switch (result) {
        case 2:  err = "No signal data found";        break;
        case 3:  err = "Unsupported frequency";       break;
        case 6:  err = "Rolling code - use RAW";      break;
        case 7:  err = "Unsupported protocol";        break;
        default: err = "Replay init failed";          break;
        }
        flipper_show_result(err, false);
        flipper_wait_for_back();
    }
}


/*============================================================================*/
/*                                                                            */
/*  NFC: IMPORT FLIPPER .NFC FILES                                           */
/*                                                                            */
/*============================================================================*/

void nfc_import_flipper(void)
{
    flipper_nfc_card_t card;

    flipper_show_status("Import NFC", "Loading...");

    flipper_ensure_dir(FLIPPER_NFC_DIR);

    /* Find first .nfc file */
    DIR dir;
    FILINFO fno;
    FRESULT fr;
    bool found = false;

    fr = f_opendir(&dir, FLIPPER_NFC_DIR);
    if (fr != FR_OK) {
        flipper_show_result("No Flipper/NFC dir", false);
        flipper_wait_for_back();
        return;
    }

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        size_t len = strlen(fno.fname);
        if (len > 4 && strcmp(&fno.fname[len - 4], ".nfc") == 0) {
            snprintf(flipper_filepath, sizeof(flipper_filepath), "%s/%s",
                     FLIPPER_NFC_DIR, fno.fname);
            found = true;
            break;
        }
    }
    f_closedir(&dir);

    if (!found) {
        flipper_show_result("No .nfc files found", false);
        flipper_wait_for_back();
        return;
    }

    /* Parse the .nfc file */
    flipper_show_status("Importing:", fno.fname);
    if (!flipper_nfc_load(flipper_filepath, &card)) {
        flipper_show_result("Parse failed", false);
        flipper_wait_for_back();
        return;
    }

    M1_LOG_I(M1_LOGDB_TAG, "NFC import: type=%d, UID len=%d, ATQA=%02X%02X, SAK=%02X\r\n",
             card.type, card.uid_len, card.atqa[0], card.atqa[1], card.sak);

    /* Populate NFC context from Flipper card data and save in M1 native format */
    {
        nfc_run_ctx_t *c = nfc_ctx_get();
        char save_path[FLIPPER_PATH_MAX];
        char base_name[64];
        bool saved = false;

        if (c)
        {
            nfc_run_ctx_init(c);

            /* Map Flipper type → M1 tech/family */
            c->head.tech = M1NFC_TECH_A;  /* Default */
            switch (card.type) {
            case FLIPPER_NFC_TYPE_ISO14443_3A:
                c->head.tech   = M1NFC_TECH_A;
                c->head.family = M1NFC_FAM_CLASSIC; /* Best guess for generic 3A */
                break;
            case FLIPPER_NFC_TYPE_MIFARE_CLASSIC:
                c->head.tech   = M1NFC_TECH_A;
                c->head.family = M1NFC_FAM_CLASSIC;
                break;
            case FLIPPER_NFC_TYPE_NTAG:
                c->head.tech   = M1NFC_TECH_A;
                c->head.family = M1NFC_FAM_ULTRALIGHT;
                break;
            case FLIPPER_NFC_TYPE_MIFARE_DESFIRE:
                c->head.tech   = M1NFC_TECH_A;
                c->head.family = M1NFC_FAM_DESFIRE;
                break;
            case FLIPPER_NFC_TYPE_ISO14443_3B:
                c->head.tech   = M1NFC_TECH_B;
                break;
            default:
                c->head.tech   = M1NFC_TECH_A;
                break;
            }

            /* Copy UID */
            c->head.uid_len = card.uid_len;
            memcpy(c->head.uid, card.uid, card.uid_len);

            /* Copy ATQA and SAK (Tech A) */
            if (c->head.tech == M1NFC_TECH_A)
            {
                memcpy(c->head.a.atqa, card.atqa, 2);
                c->head.a.has_atqa = true;
                c->head.a.sak     = card.sak;
                c->head.a.has_sak = true;
            }

            /* Ensure 0:/NFC/ directory exists */
            fs_directory_ensure(M1_NFC_SAVE_DIR);

            /* Derive save filename from source (strip .nfc, re-add .nfc) */
            strncpy(base_name, fno.fname, sizeof(base_name) - 1);
            base_name[sizeof(base_name) - 1] = '\0';
            size_t blen = strlen(base_name);
            if (blen > 4 && strcmp(&base_name[blen - 4], ".nfc") == 0)
                base_name[blen - 4] = '\0';

            snprintf(save_path, sizeof(save_path), "%s/%s.nfc",
                     M1_NFC_SAVE_DIR, base_name);

            saved = nfc_profile_save(save_path, c);
            M1_LOG_I(M1_LOGDB_TAG, "NFC save to %s: %s\r\n",
                     save_path, saved ? "OK" : "FAIL");
        }

        /* Display result */
        char uid_str[32] = {0};
        char info_str[48] = {0};
        uint8_t i;

        for (i = 0; i < card.uid_len && i < 7; i++) {
            snprintf(&uid_str[i * 3], sizeof(uid_str) - i * 3, "%02X ", card.uid[i]);
        }
        snprintf(info_str, sizeof(info_str), "UID: %s", uid_str);

        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, saved ? "NFC Saved" : "NFC Save Failed");
            u8g2_DrawStr(&m1_u8g2, 2, 28, info_str);
            u8g2_DrawStr(&m1_u8g2, 2, 44, fno.fname);
            u8g2_DrawStr(&m1_u8g2, 2, FLIPPER_STATUS_LINE_Y, "Press BACK to exit");
        } while (u8g2_NextPage(&m1_u8g2));
    }

    flipper_wait_for_back();
}


/*============================================================================*/
/*                                                                            */
/*  RFID: IMPORT FLIPPER .RFID FILES                                         */
/*                                                                            */
/*============================================================================*/

void rfid_import_flipper(void)
{
    flipper_rfid_tag_t tag;

    flipper_show_status("Import RFID", "Loading...");

    flipper_ensure_dir(FLIPPER_RFID_DIR);

    /* Find first .rfid file */
    DIR dir;
    FILINFO fno;
    FRESULT fr;
    bool found = false;

    fr = f_opendir(&dir, FLIPPER_RFID_DIR);
    if (fr != FR_OK) {
        flipper_show_result("No Flipper/RFID dir", false);
        flipper_wait_for_back();
        return;
    }

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        size_t len = strlen(fno.fname);
        if (len > 5 && strcmp(&fno.fname[len - 5], ".rfid") == 0) {
            snprintf(flipper_filepath, sizeof(flipper_filepath), "%s/%s",
                     FLIPPER_RFID_DIR, fno.fname);
            found = true;
            break;
        }
    }
    f_closedir(&dir);

    if (!found) {
        flipper_show_result("No .rfid files found", false);
        flipper_wait_for_back();
        return;
    }

    /* Parse the .rfid file */
    flipper_show_status("Importing:", fno.fname);
    if (!flipper_rfid_load(flipper_filepath, &tag)) {
        flipper_show_result("Parse failed", false);
        flipper_wait_for_back();
        return;
    }

    M1_LOG_I(M1_LOGDB_TAG, "RFID import: protocol=%d, data_len=%d\r\n",
             tag.protocol, tag.data_len);

    /* Convert Flipper tag → M1 LFRFID_TAG_INFO and save */
    {
        LFRFID_TAG_INFO tag_info;
        char save_path[FLIPPER_PATH_MAX];
        char base_name[64];
        bool saved = false;

        memset(&tag_info, 0, sizeof(tag_info));
        tag_info.protocol = (uint8_t)tag.protocol;

        /* Copy data → uid (M1 uses uid[5], copy up to available) */
        uint8_t copy_len = tag.data_len;
        if (copy_len > sizeof(tag_info.uid))
            copy_len = sizeof(tag_info.uid);
        memcpy(tag_info.uid, tag.data, copy_len);

        /* Set bitrate based on protocol (matches lfrfid_profile_load logic) */
        if (tag_info.protocol == 0)       /* EM4100 */
            tag_info.bitrate = 64;
        else if (tag_info.protocol == 1)  /* H10301 */
            tag_info.bitrate = 32;
        else if (tag_info.protocol == 2)  /* HID Generic */
            tag_info.bitrate = 16;

        /* Ensure 0:/RFID/ directory exists */
        fs_directory_ensure(M1_RFID_SAVE_DIR);

        /* Derive save filename from source (strip .rfid, re-add .rfid) */
        strncpy(base_name, fno.fname, sizeof(base_name) - 1);
        base_name[sizeof(base_name) - 1] = '\0';
        size_t blen = strlen(base_name);
        if (blen > 5 && strcmp(&base_name[blen - 5], ".rfid") == 0)
            base_name[blen - 5] = '\0';

        snprintf(save_path, sizeof(save_path), "%s/%s.rfid",
                 M1_RFID_SAVE_DIR, base_name);

        saved = lfrfid_profile_save(save_path, &tag_info);
        M1_LOG_I(M1_LOGDB_TAG, "RFID save to %s: %s\r\n",
                 save_path, saved ? "OK" : "FAIL");

        /* Display result */
        char data_str[32] = {0};
        char info_str[48] = {0};
        uint8_t i;

        for (i = 0; i < tag.data_len && i < 8; i++) {
            snprintf(&data_str[i * 3], sizeof(data_str) - i * 3, "%02X ", tag.data[i]);
        }
        snprintf(info_str, sizeof(info_str), "Data: %s", data_str);

        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, saved ? "RFID Saved" : "RFID Save Failed");
            u8g2_DrawStr(&m1_u8g2, 2, 28, info_str);
            u8g2_DrawStr(&m1_u8g2, 2, 44, fno.fname);
            u8g2_DrawStr(&m1_u8g2, 2, FLIPPER_STATUS_LINE_Y, "Press BACK to exit");
        } while (u8g2_NextPage(&m1_u8g2));
    }

    flipper_wait_for_back();
}

#else /* M1_APP_FILE_IMPORT_ENABLE not defined */

/* Stub implementations when file import is disabled */
void sub_ghz_replay_flipper(void) { }
void nfc_import_flipper(void) { }
void rfid_import_flipper(void) { }

#endif /* M1_APP_FILE_IMPORT_ENABLE */
