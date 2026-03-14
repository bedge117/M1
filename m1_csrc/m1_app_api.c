/* See COPYING.txt for license details. */

/*
*
*  m1_app_api.c
*
*  Firmware API symbol table for M1 external apps (.m1app)
*
*  This module builds a sorted hash table of firmware function pointers
*  that external apps can call. At init time, hashes are computed from
*  symbol name strings, the table is sorted, and a binary search is
*  used at load time to resolve external references.
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "m1_elf_loader.h"
#include "m1_app_api.h"
#include "m1_games.h"
#include "m1_log_debug.h"
#include "u8g2.h"
#include "ff.h"
#include "m1_buzzer.h"
#include "m1_gpio.h"
#include "m1_lp5814.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_file_util.h"
#include "m1_crypto.h"
#include "m1_i2c.h"
#include "m1_rf_spi.h"
#include "m1_lib.h"
#include "m1_virtual_kb.h"
#include "m1_power_ctl.h"
#include "m1_sdcard.h"
#include "m1_infrared.h"
#include "flipper_file.h"
#include "flipper_ir.h"
#include "flipper_nfc.h"
#include "flipper_rfid.h"
#include "flipper_subghz.h"
#include "m1_wifi.h"
#include "m1_field_detect.h"

/*************************** D E F I N E S ************************************/

#define TAG "API"

/* Maximum number of exported API symbols */
#define API_MAX_SYMBOLS  200

/********************* T Y P E S *********************************************/

/* Entry used during initialization (name string + function pointer) */
typedef struct {
    const char *name;
    void       *address;
} api_name_entry_t;

/***************************** V A R I A B L E S ******************************/

/* The final sorted hash table used at runtime */
static m1_api_sym_t s_api_table[API_MAX_SYMBOLS];
static uint16_t s_api_count = 0;
static bool s_api_initialized = false;

static m1_api_interface_t s_api_interface = {
    .table = s_api_table,
    .count = 0,
    .api_version = M1_APP_API_VERSION
};

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void api_sort_table(void);

/*************** W R A P P E R   F U N C T I O N S ***************************/

/*
 * Wrapper for m1_u8g2_nextpage — apps call this as "m1app_display_flush"
 * to push their framebuffer to the display.
 */
static uint8_t m1app_display_flush(void)
{
    return m1_u8g2_nextpage();
}

/*
 * Wrapper for m1_u8g2_firstpage
 */
static void m1app_display_begin(void)
{
    m1_u8g2_firstpage();
}

/*
 * Wrapper to give apps access to the u8g2 handle
 */
static u8g2_t *m1app_get_u8g2(void)
{
    return &m1_u8g2;
}

/*
 * Wrapper for pvPortMalloc — apps call "m1app_malloc"
 */
static void *m1app_malloc(size_t size)
{
    return pvPortMalloc(size);
}

/*
 * Wrapper for vPortFree — apps call "m1app_free"
 */
static void m1app_free(void *ptr)
{
    vPortFree(ptr);
}

/*
 * Wrapper for vTaskDelay in ms
 */
static void m1app_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*
 * Wrapper for HAL_GetTick
 */
static uint32_t m1app_get_tick(void)
{
    return HAL_GetTick();
}

/*
 * Wrappers for u8g2 macros (can't export macros via symbol table)
 */
static int8_t m1app_u8g2_get_ascent(u8g2_t *u8g2)
{
    return u8g2_GetAscent(u8g2);
}

static int8_t m1app_u8g2_get_descent(u8g2_t *u8g2)
{
    return u8g2_GetDescent(u8g2);
}

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Initialize the API table: compute hashes and sort
 */
/*============================================================================*/
void m1_app_api_init(void)
{
    uint16_t i;

    if (s_api_initialized)
        return;

    /*
     * Define all exported symbols. The string name is what the app's
     * compiler will emit as an external reference. The address is the
     * actual function pointer in firmware.
     */
    static const api_name_entry_t entries[] = {

        /* ===== Display — u8g2 drawing functions ===== */
        { "u8g2_FirstPage",    (void *)u8g2_FirstPage },
        { "u8g2_DrawStr",      (void *)u8g2_DrawStr },
        { "u8g2_DrawBox",      (void *)u8g2_DrawBox },
        { "u8g2_DrawFrame",    (void *)u8g2_DrawFrame },
        { "u8g2_DrawPixel",    (void *)u8g2_DrawPixel },
        { "u8g2_DrawLine",     (void *)u8g2_DrawLine },
        { "u8g2_DrawCircle",   (void *)u8g2_DrawCircle },
        { "u8g2_DrawDisc",     (void *)u8g2_DrawDisc },
        { "u8g2_DrawHLine",    (void *)u8g2_DrawHLine },
        { "u8g2_DrawVLine",    (void *)u8g2_DrawVLine },
        { "u8g2_DrawRBox",     (void *)u8g2_DrawRBox },
        { "u8g2_DrawRFrame",   (void *)u8g2_DrawRFrame },
        { "u8g2_DrawTriangle", (void *)u8g2_DrawTriangle },
        { "u8g2_DrawXBM",      (void *)u8g2_DrawXBM },
        { "u8g2_SetDrawColor", (void *)u8g2_SetDrawColor },
        { "u8g2_SetFont",      (void *)u8g2_SetFont },
        { "u8g2_GetStrWidth",  (void *)u8g2_GetStrWidth },
        { "u8g2_GetAscent",    (void *)m1app_u8g2_get_ascent },
        { "u8g2_GetDescent",   (void *)m1app_u8g2_get_descent },
        { "u8g2_SetFontDirection", (void *)u8g2_SetFontDirection },
        { "u8g2_SetClipWindow",    (void *)u8g2_SetClipWindow },
        { "u8g2_SetMaxClipWindow", (void *)u8g2_SetMaxClipWindow },

        /* ===== Display — M1 wrappers ===== */
        { "m1app_display_begin",  (void *)m1app_display_begin },
        { "m1app_display_flush",  (void *)m1app_display_flush },
        { "m1app_get_u8g2",       (void *)m1app_get_u8g2 },
        { "m1_lcd_cleardisplay",  (void *)m1_lcd_cleardisplay },

        /* ===== Display — M1 UI helpers ===== */
        { "m1_message_box",       (void *)m1_message_box },
        { "m1_draw_bottom_bar",   (void *)m1_draw_bottom_bar },
        { "m1_draw_icon",         (void *)m1_draw_icon },
        { "m1_draw_text",         (void *)m1_draw_text },
        { "m1_draw_text_box",     (void *)m1_draw_text_box },
        { "m1_info_box_display_init",  (void *)m1_info_box_display_init },
        { "m1_info_box_display_clear", (void *)m1_info_box_display_clear },
        { "m1_info_box_display_draw",  (void *)m1_info_box_display_draw },

        /* ===== Timing and delays ===== */
        { "m1app_delay",       (void *)m1app_delay },
        { "m1app_get_tick",    (void *)m1app_get_tick },
        { "vTaskDelay",        (void *)vTaskDelay },
        { "HAL_GetTick",       (void *)HAL_GetTick },
        { "m1_hard_delay",     (void *)m1_hard_delay },

        /* ===== Memory management ===== */
        { "m1app_malloc",      (void *)m1app_malloc },
        { "m1app_free",        (void *)m1app_free },
        { "pvPortMalloc",      (void *)pvPortMalloc },
        { "vPortFree",         (void *)vPortFree },

        /* ===== Input ===== */
        { "game_poll_button",  (void *)game_poll_button },
        { "game_rand_seed",    (void *)game_rand_seed },
        { "game_rand_range",   (void *)game_rand_range },

        /* ===== Virtual keyboard ===== */
        { "m1_vkb_get_filename", (void *)m1_vkb_get_filename },
        { "m1_vkbs_get_data",    (void *)m1_vkbs_get_data },

        /* ===== Audio / Buzzer ===== */
        { "m1_buzzer_notification",  (void *)m1_buzzer_notification },
        { "m1_buzzer_notification2", (void *)m1_buzzer_notification2 },
        { "m1_buzzer_set",           (void *)m1_buzzer_set },

        /* ===== LED (RGB notification LED) ===== */
        { "lp5814_led_on_Red",       (void *)lp5814_led_on_Red },
        { "lp5814_led_on_Green",     (void *)lp5814_led_on_Green },
        { "lp5814_led_on_Blue",      (void *)lp5814_led_on_Blue },
        { "lp5814_led_on_rgb",       (void *)lp5814_led_on_rgb },
        { "lp5814_all_off_RGB",      (void *)lp5814_all_off_RGB },
        { "lp5814_backlight_on",     (void *)lp5814_backlight_on },
        { "lp5814_fastblink_on_R_G_B", (void *)lp5814_fastblink_on_R_G_B },
        { "lp5814_led_on",           (void *)lp5814_led_on },
        { "lp5814_led_off",          (void *)lp5814_led_off },

        /* ===== GPIO ===== */
        { "ext_power_5V_set",      (void *)ext_power_5V_set },
        { "ext_power_3V_set",      (void *)ext_power_3V_set },
        { "HAL_GPIO_ReadPin",      (void *)HAL_GPIO_ReadPin },
        { "HAL_GPIO_WritePin",     (void *)HAL_GPIO_WritePin },
        { "HAL_GPIO_TogglePin",    (void *)HAL_GPIO_TogglePin },

        /* ===== I2C ===== */
        { "m1_i2c_hal_trans_req",  (void *)m1_i2c_hal_trans_req },
        { "m1_i2c_hal_get_error",  (void *)m1_i2c_hal_get_error },

        /* ===== SPI ===== */
        { "m1_spi_hal_trans_req",  (void *)m1_spi_hal_trans_req },

        /* ===== Infrared ===== */
        { "infrared_encode_sys_init",   (void *)infrared_encode_sys_init },
        { "infrared_encode_sys_deinit", (void *)infrared_encode_sys_deinit },
        { "infrared_transmit",          (void *)infrared_transmit },

        /* ===== Power / Battery ===== */
        { "m1_check_battery_level", (void *)m1_check_battery_level },

        /* ===== SD card info ===== */
        { "m1_sdcard_get_status",        (void *)m1_sdcard_get_status },
        { "m1_sd_detected",              (void *)m1_sd_detected },
        { "m1_sdcard_get_free_capacity", (void *)m1_sdcard_get_free_capacity },
        { "m1_sdcard_get_total_capacity",(void *)m1_sdcard_get_total_capacity },

        /* ===== FatFS — file operations ===== */
        { "f_open",     (void *)f_open },
        { "f_close",    (void *)f_close },
        { "f_read",     (void *)f_read },
        { "f_write",    (void *)f_write },
        { "f_lseek",    (void *)f_lseek },
        { "f_sync",     (void *)f_sync },
        { "f_truncate", (void *)f_truncate },
        { "f_printf",   (void *)f_printf },
        { "f_puts",     (void *)f_puts },

        /* ===== FatFS — directory operations ===== */
        { "f_opendir",  (void *)f_opendir },
        { "f_closedir", (void *)f_closedir },
        { "f_readdir",  (void *)f_readdir },
        { "f_mkdir",    (void *)f_mkdir },
        { "f_unlink",   (void *)f_unlink },
        { "f_rename",   (void *)f_rename },
        { "f_stat",     (void *)f_stat },

        /* ===== File utilities ===== */
        { "fu_get_filename",             (void *)fu_get_filename },
        { "fu_get_file_extension",       (void *)fu_get_file_extension },
        { "fu_get_filename_without_ext", (void *)fu_get_filename_without_ext },
        { "fu_get_directory_path",       (void *)fu_get_directory_path },
        { "fu_path_combine",             (void *)fu_path_combine },
        { "fs_file_exists",              (void *)fs_file_exists },
        { "fs_directory_exists",         (void *)fs_directory_exists },
        { "fs_directory_ensure",         (void *)fs_directory_ensure },

        /* ===== Flipper file format parser ===== */
        { "ff_open",              (void *)ff_open },
        { "ff_open_write",        (void *)ff_open_write },
        { "ff_close",             (void *)ff_close },
        { "ff_read_line",         (void *)ff_read_line },
        { "ff_parse_kv",          (void *)ff_parse_kv },
        { "ff_get_key",           (void *)ff_get_key },
        { "ff_get_value",         (void *)ff_get_value },
        { "ff_is_separator",      (void *)ff_is_separator },
        { "ff_validate_header",   (void *)ff_validate_header },
        { "ff_write_kv_str",      (void *)ff_write_kv_str },
        { "ff_write_kv_uint32",   (void *)ff_write_kv_uint32 },
        { "ff_write_kv_hex",      (void *)ff_write_kv_hex },
        { "ff_write_separator",   (void *)ff_write_separator },
        { "ff_write_comment",     (void *)ff_write_comment },
        { "ff_parse_hex_bytes",   (void *)ff_parse_hex_bytes },
        { "ff_parse_int32_array", (void *)ff_parse_int32_array },

        /* ===== Flipper IR file format ===== */
        { "flipper_ir_open",          (void *)flipper_ir_open },
        { "flipper_ir_read_signal",   (void *)flipper_ir_read_signal },
        { "flipper_ir_write_header",  (void *)flipper_ir_write_header },
        { "flipper_ir_write_signal",  (void *)flipper_ir_write_signal },
        { "flipper_ir_count_signals", (void *)flipper_ir_count_signals },
        { "flipper_ir_proto_to_irmp", (void *)flipper_ir_proto_to_irmp },
        { "flipper_ir_irmp_to_proto", (void *)flipper_ir_irmp_to_proto },

        /* ===== Flipper NFC file format ===== */
        { "flipper_nfc_load",       (void *)flipper_nfc_load },
        { "flipper_nfc_save",       (void *)flipper_nfc_save },
        { "flipper_nfc_parse_type", (void *)flipper_nfc_parse_type },

        /* ===== Flipper RFID file format ===== */
        { "flipper_rfid_load",           (void *)flipper_rfid_load },
        { "flipper_rfid_save",           (void *)flipper_rfid_save },
        { "flipper_rfid_parse_protocol", (void *)flipper_rfid_parse_protocol },

        /* ===== Flipper Sub-GHz file format ===== */
        { "flipper_subghz_load", (void *)flipper_subghz_load },
        { "flipper_subghz_save", (void *)flipper_subghz_save },

        /* ===== Crypto (AES-256-CBC) ===== */
        { "m1_crypto_derive_key",  (void *)m1_crypto_derive_key },
        { "m1_crypto_generate_iv", (void *)m1_crypto_generate_iv },
        { "m1_crypto_encrypt",     (void *)m1_crypto_encrypt },
        { "m1_crypto_decrypt",     (void *)m1_crypto_decrypt },

        /* ===== String utilities ===== */
        { "m1_strtolower",      (void *)m1_strtolower },
        { "m1_strtoupper",      (void *)m1_strtoupper },
        { "m1_byte_to_hextext", (void *)m1_byte_to_hextext },
        { "m1_float_to_string", (void *)m1_float_to_string },

        /* ===== u8g2 font data ===== */
        { "u8g2_font_4x6_tr",              (void *)u8g2_font_4x6_tr },
        { "u8g2_font_5x8_tr",              (void *)u8g2_font_5x8_tr },
        { "u8g2_font_6x10_tr",             (void *)u8g2_font_6x10_tr },
        { "u8g2_font_10x20_mr",            (void *)u8g2_font_10x20_mr },
        { "u8g2_font_helvB08_tr",          (void *)u8g2_font_helvB08_tr },
        { "u8g2_font_helvB08_tf",          (void *)u8g2_font_helvB08_tf },
        { "u8g2_font_finderskeepers_tf",   (void *)u8g2_font_finderskeepers_tf },
        { "u8g2_font_resoledmedium_tr",    (void *)u8g2_font_resoledmedium_tr },
        { "u8g2_font_NokiaSmallPlain_tf",  (void *)u8g2_font_NokiaSmallPlain_tf },
        { "u8g2_font_squeezed_b7_tr",      (void *)u8g2_font_squeezed_b7_tr },
        { "u8g2_font_spleen5x8_mf",        (void *)u8g2_font_spleen5x8_mf },
        { "u8g2_font_spleen8x16_mf",       (void *)u8g2_font_spleen8x16_mf },
        { "u8g2_font_nine_by_five_nbp_tf", (void *)u8g2_font_nine_by_five_nbp_tf },
        { "u8g2_font_courB08_tf",          (void *)u8g2_font_courB08_tf },
        { "u8g2_font_Terminal_tr",          (void *)u8g2_font_Terminal_tr },
        { "u8g2_font_lubB08_tf",           (void *)u8g2_font_lubB08_tf },
        { "u8g2_font_profont17_tr",         (void *)u8g2_font_profont17_tr },
        { "u8g2_font_VCR_OSD_tu",          (void *)u8g2_font_VCR_OSD_tu },
        { "u8g2_font_Pixellari_tu",         (void *)u8g2_font_Pixellari_tu },
        { "u8g2_font_pcsenior_8f",          (void *)u8g2_font_pcsenior_8f },

        /* ===== Field Detection ===== */
        { "m1_field_detect_start",    (void *)m1_field_detect_start },
        { "m1_field_detect_stop",     (void *)m1_field_detect_stop },
        { "m1_field_detect_nfc",      (void *)m1_field_detect_nfc },
        { "m1_field_detect_rfid",     (void *)m1_field_detect_rfid },
        { "m1_field_detect_rfid_raw", (void *)m1_field_detect_rfid_raw },
        { "m1_field_detect_nfc_raw",  (void *)m1_field_detect_nfc_raw },
        { "m1_field_detect_nfc_opctl",(void *)m1_field_detect_nfc_opctl },

        /* ===== C library ===== */
        { "memset",    (void *)memset },
        { "memcpy",    (void *)memcpy },
        { "memcmp",    (void *)memcmp },
        { "strlen",    (void *)strlen },
        { "strcmp",     (void *)strcmp },
        { "strncmp",   (void *)strncmp },
        { "strncpy",   (void *)strncpy },
        { "strncat",   (void *)strncat },
        { "strstr",    (void *)strstr },
        { "strchr",    (void *)strchr },
        { "snprintf",  (void *)snprintf },
        { "atoi",      (void *)atoi },
        { "strtol",    (void *)strtol },
        { "rand",      (void *)rand },
        { "srand",     (void *)srand },
        { "abs",       (void *)abs },
    };

    uint16_t num_entries = sizeof(entries) / sizeof(entries[0]);

    if (num_entries > API_MAX_SYMBOLS)
    {
        M1_LOG_E(TAG, "Too many API symbols (%u > %u)", num_entries, API_MAX_SYMBOLS);
        num_entries = API_MAX_SYMBOLS;
    }

    /* Compute hashes and populate the table */
    for (i = 0; i < num_entries; i++)
    {
        s_api_table[i].hash = elf_gnu_hash(entries[i].name);
        s_api_table[i].address = entries[i].address;
    }

    s_api_count = num_entries;

    /* Sort by hash for binary search */
    api_sort_table();

    s_api_interface.table = s_api_table;
    s_api_interface.count = s_api_count;
    s_api_interface.api_version = M1_APP_API_VERSION;

    s_api_initialized = true;

    M1_LOG_I(TAG, "API initialized: %u symbols, version %u",
             s_api_count, M1_APP_API_VERSION);
}


/*============================================================================*/
/*
 * @brief  Sort the API table by hash value (insertion sort — small table)
 */
/*============================================================================*/
static void api_sort_table(void)
{
    uint16_t i, j;
    m1_api_sym_t temp;

    for (i = 1; i < s_api_count; i++)
    {
        temp = s_api_table[i];
        j = i;
        while (j > 0 && s_api_table[j - 1].hash > temp.hash)
        {
            s_api_table[j] = s_api_table[j - 1];
            j--;
        }
        s_api_table[j] = temp;
    }

    /* Check for hash collisions (debug build only) */
    for (i = 1; i < s_api_count; i++)
    {
        if (s_api_table[i].hash == s_api_table[i - 1].hash)
        {
            M1_LOG_W(TAG, "Hash collision at index %u (hash=0x%08lX)",
                     i, (unsigned long)s_api_table[i].hash);
        }
    }
}


/*============================================================================*/
/*
 * @brief  Get the firmware API interface
 * @retval const m1_api_interface_t*  Pointer to the API interface
 */
/*============================================================================*/
const m1_api_interface_t *m1_app_get_api(void)
{
    if (!s_api_initialized)
    {
        m1_app_api_init();
    }
    return &s_api_interface;
}
