/* See COPYING.txt for license details. */

/*
 * m1_badbt.c
 *
 * Bad-BT — BLE HID keyboard injection with DuckyScript parser.
 * Reads script files from SD card and types keystrokes via BLE HID.
 * Same DuckyScript format as BadUSB, but wireless over Bluetooth.
 */

/*************************** I N C L U D E S **********************************/
#include "app_freertos.h"
#include "cmsis_os.h"
#include "main.h"
#include "m1_lcd.h"
#include "m1_display.h"
#include "m1_file_browser.h"
#include "m1_menu.h"
#include "m1_tasks.h"
#include "m1_system.h"
#include "m1_sdcard.h"
#include "m1_compile_cfg.h"
#include "m1_badbt.h"
#include "m1_esp32_hal.h"
#include "esp_app_main.h"
#include "ctrl_api.h"
#include "m1_log_debug.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifdef M1_APP_BADBT_ENABLE

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG  "BBT"

/* Timing */
#define BADBT_KEY_PRESS_MS        30    /* Hold key down (BLE is slower than USB) */
#define BADBT_KEY_RELEASE_MS      30    /* Delay after release */
#define BADBT_INTER_CHAR_MS       10    /* Between characters in STRING */
#define BADBT_CONNECT_TIMEOUT     120   /* Seconds to wait for BLE connection */

/* HID Modifier bits (same as USB HID) */
#define HID_MOD_LCTRL   0x01
#define HID_MOD_LSHIFT  0x02
#define HID_MOD_LALT    0x04
#define HID_MOD_LGUI    0x08

/* HID Keyboard scancodes */
#define KEY_NONE                  0x00
#define KEY_A                     0x04
#define KEY_B                     0x05
#define KEY_C                     0x06
#define KEY_D                     0x07
#define KEY_E                     0x08
#define KEY_F                     0x09
#define KEY_G                     0x0A
#define KEY_H                     0x0B
#define KEY_I                     0x0C
#define KEY_J                     0x0D
#define KEY_K                     0x0E
#define KEY_L                     0x0F
#define KEY_M                     0x10
#define KEY_N                     0x11
#define KEY_O                     0x12
#define KEY_P                     0x13
#define KEY_Q                     0x14
#define KEY_R                     0x15
#define KEY_S                     0x16
#define KEY_T                     0x17
#define KEY_U                     0x18
#define KEY_V                     0x19
#define KEY_W                     0x1A
#define KEY_X                     0x1B
#define KEY_Y                     0x1C
#define KEY_Z                     0x1D
#define KEY_1                     0x1E
#define KEY_2                     0x1F
#define KEY_3                     0x20
#define KEY_4                     0x21
#define KEY_5                     0x22
#define KEY_6                     0x23
#define KEY_7                     0x24
#define KEY_8                     0x25
#define KEY_9                     0x26
#define KEY_0                     0x27
#define KEY_ENTER                 0x28
#define KEY_ESCAPE                0x29
#define KEY_BACKSPACE             0x2A
#define KEY_TAB                   0x2B
#define KEY_SPACE                 0x2C
#define KEY_MINUS                 0x2D
#define KEY_EQUAL                 0x2E
#define KEY_LEFTBRACE             0x2F
#define KEY_RIGHTBRACE            0x30
#define KEY_BACKSLASH             0x31
#define KEY_SEMICOLON             0x33
#define KEY_APOSTROPHE            0x34
#define KEY_GRAVE                 0x35
#define KEY_COMMA                 0x36
#define KEY_DOT                   0x37
#define KEY_SLASH                 0x38
#define KEY_CAPSLOCK              0x39
#define KEY_F1                    0x3A
#define KEY_F2                    0x3B
#define KEY_F3                    0x3C
#define KEY_F4                    0x3D
#define KEY_F5                    0x3E
#define KEY_F6                    0x3F
#define KEY_F7                    0x40
#define KEY_F8                    0x41
#define KEY_F9                    0x42
#define KEY_F10                   0x43
#define KEY_F11                   0x44
#define KEY_F12                   0x45
#define KEY_PRINTSCREEN           0x46
#define KEY_SCROLLLOCK            0x47
#define KEY_PAUSE                 0x48
#define KEY_INSERT                0x49
#define KEY_HOME                  0x4A
#define KEY_PAGEUP                0x4B
#define KEY_DELETE                0x4C
#define KEY_END                   0x4D
#define KEY_PAGEDOWN              0x4E
#define KEY_RIGHT                 0x4F
#define KEY_LEFT                  0x50
#define KEY_DOWN                  0x51
#define KEY_UP                    0x52
#define KEY_NUMLOCK               0x53
#define KEY_MENU                  0x65

/************************** S T R U C T U R E S *******************************/

/* ASCII to HID scancode + shift modifier mapping */
typedef struct
{
    uint8_t keycode;
    uint8_t shift;    /* 1 = shift required */
} ascii_hid_map_t;

/***************************** V A R I A B L E S ******************************/

static badbt_state_t badbt_state;
static ctrl_cmd_t badbt_req;  /* initialized via CTRL_CMD_DEFAULT_REQ() before use */

/* ASCII to HID scancode table (US keyboard layout, indices 0x20-0x7E) */
static const ascii_hid_map_t ascii_to_hid[] =
{
    /* 0x20 ' '  */ {KEY_SPACE,      0},
    /* 0x21 '!'  */ {KEY_1,          1},
    /* 0x22 '"'  */ {KEY_APOSTROPHE, 1},
    /* 0x23 '#'  */ {KEY_3,          1},
    /* 0x24 '$'  */ {KEY_4,          1},
    /* 0x25 '%'  */ {KEY_5,          1},
    /* 0x26 '&'  */ {KEY_7,          1},
    /* 0x27 '\'' */ {KEY_APOSTROPHE, 0},
    /* 0x28 '('  */ {KEY_9,          1},
    /* 0x29 ')'  */ {KEY_0,          1},
    /* 0x2A '*'  */ {KEY_8,          1},
    /* 0x2B '+'  */ {KEY_EQUAL,      1},
    /* 0x2C ','  */ {KEY_COMMA,      0},
    /* 0x2D '-'  */ {KEY_MINUS,      0},
    /* 0x2E '.'  */ {KEY_DOT,        0},
    /* 0x2F '/'  */ {KEY_SLASH,      0},
    /* 0x30 '0'  */ {KEY_0,          0},
    /* 0x31 '1'  */ {KEY_1,          0},
    /* 0x32 '2'  */ {KEY_2,          0},
    /* 0x33 '3'  */ {KEY_3,          0},
    /* 0x34 '4'  */ {KEY_4,          0},
    /* 0x35 '5'  */ {KEY_5,          0},
    /* 0x36 '6'  */ {KEY_6,          0},
    /* 0x37 '7'  */ {KEY_7,          0},
    /* 0x38 '8'  */ {KEY_8,          0},
    /* 0x39 '9'  */ {KEY_9,          0},
    /* 0x3A ':'  */ {KEY_SEMICOLON,  1},
    /* 0x3B ';'  */ {KEY_SEMICOLON,  0},
    /* 0x3C '<'  */ {KEY_COMMA,      1},
    /* 0x3D '='  */ {KEY_EQUAL,      0},
    /* 0x3E '>'  */ {KEY_DOT,        1},
    /* 0x3F '?'  */ {KEY_SLASH,      1},
    /* 0x40 '@'  */ {KEY_2,          1},
    /* 0x41-0x5A: A-Z */
    {KEY_A, 1}, {KEY_B, 1}, {KEY_C, 1}, {KEY_D, 1}, {KEY_E, 1},
    {KEY_F, 1}, {KEY_G, 1}, {KEY_H, 1}, {KEY_I, 1}, {KEY_J, 1},
    {KEY_K, 1}, {KEY_L, 1}, {KEY_M, 1}, {KEY_N, 1}, {KEY_O, 1},
    {KEY_P, 1}, {KEY_Q, 1}, {KEY_R, 1}, {KEY_S, 1}, {KEY_T, 1},
    {KEY_U, 1}, {KEY_V, 1}, {KEY_W, 1}, {KEY_X, 1}, {KEY_Y, 1},
    {KEY_Z, 1},
    /* 0x5B '['  */ {KEY_LEFTBRACE,  0},
    /* 0x5C '\\' */ {KEY_BACKSLASH,  0},
    /* 0x5D ']'  */ {KEY_RIGHTBRACE, 0},
    /* 0x5E '^'  */ {KEY_6,          1},
    /* 0x5F '_'  */ {KEY_MINUS,      1},
    /* 0x60 '`'  */ {KEY_GRAVE,      0},
    /* 0x61-0x7A: a-z */
    {KEY_A, 0}, {KEY_B, 0}, {KEY_C, 0}, {KEY_D, 0}, {KEY_E, 0},
    {KEY_F, 0}, {KEY_G, 0}, {KEY_H, 0}, {KEY_I, 0}, {KEY_J, 0},
    {KEY_K, 0}, {KEY_L, 0}, {KEY_M, 0}, {KEY_N, 0}, {KEY_O, 0},
    {KEY_P, 0}, {KEY_Q, 0}, {KEY_R, 0}, {KEY_S, 0}, {KEY_T, 0},
    {KEY_U, 0}, {KEY_V, 0}, {KEY_W, 0}, {KEY_X, 0}, {KEY_Y, 0},
    {KEY_Z, 0},
    /* 0x7B '{'  */ {KEY_LEFTBRACE,  1},
    /* 0x7C '|'  */ {KEY_BACKSLASH,  1},
    /* 0x7D '}'  */ {KEY_RIGHTBRACE, 1},
    /* 0x7E '~'  */ {KEY_GRAVE,      1},
};


/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void badbt_send_key(uint8_t modifier, uint8_t keycode);
static void badbt_release_all(void);
static void badbt_type_char(char c);
static void badbt_type_string(const char *str);
static bool badbt_parse_line(const char *line);
static uint8_t badbt_parse_key_name(const char *name);
static uint8_t badbt_parse_modifier(const char *name, const char **remainder);
static uint16_t badbt_count_lines(const char *buf, uint32_t len);
static void badbt_show_progress(const char *filename);
static bool badbt_check_abort(void);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
  * @brief  Send a key press via BLE HID, wait, then release
  */
/*============================================================================*/
static void badbt_send_key(uint8_t modifier, uint8_t keycode)
{
    ble_hid_send_kb(&badbt_req, modifier, keycode);
    osDelay(BADBT_KEY_PRESS_MS);

    badbt_release_all();
    osDelay(BADBT_KEY_RELEASE_MS);
}


/*============================================================================*/
/**
  * @brief  Release all keys (send empty HID report)
  */
/*============================================================================*/
static void badbt_release_all(void)
{
    ble_hid_send_kb(&badbt_req, 0, 0);
}


/*============================================================================*/
/**
  * @brief  Type a single ASCII character via BLE HID keyboard
  */
/*============================================================================*/
static void badbt_type_char(char c)
{
    if (c < 0x20 || c > 0x7E)
    {
        if (c == '\n' || c == '\r')
        {
            badbt_send_key(0, KEY_ENTER);
        }
        else if (c == '\t')
        {
            badbt_send_key(0, KEY_TAB);
        }
        return;
    }

    const ascii_hid_map_t *entry = &ascii_to_hid[c - 0x20];
    uint8_t mod = entry->shift ? HID_MOD_LSHIFT : 0;
    badbt_send_key(mod, entry->keycode);
}


/*============================================================================*/
/**
  * @brief  Type a string character by character via BLE HID
  */
/*============================================================================*/
static void badbt_type_string(const char *str)
{
    uint8_t count = 0;
    while (*str && badbt_state.running)
    {
        badbt_type_char(*str++);
        osDelay(BADBT_INTER_CHAR_MS);

        /* Check for abort every 4 characters */
        if (++count >= 4)
        {
            count = 0;
            if (badbt_check_abort())
            {
                badbt_state.running = 0;
                break;
            }
        }
    }
}


/*============================================================================*/
/**
  * @brief  Parse a named key to HID scancode
  * @retval HID keycode or KEY_NONE if not recognized
  */
/*============================================================================*/
static uint8_t badbt_parse_key_name(const char *name)
{
    if (strcmp(name, "ENTER") == 0 || strcmp(name, "RETURN") == 0)  return KEY_ENTER;
    if (strcmp(name, "TAB") == 0)           return KEY_TAB;
    if (strcmp(name, "ESCAPE") == 0 || strcmp(name, "ESC") == 0)    return KEY_ESCAPE;
    if (strcmp(name, "SPACE") == 0)         return KEY_SPACE;
    if (strcmp(name, "BACKSPACE") == 0)     return KEY_BACKSPACE;
    if (strcmp(name, "DELETE") == 0 || strcmp(name, "DEL") == 0)    return KEY_DELETE;
    if (strcmp(name, "INSERT") == 0)        return KEY_INSERT;
    if (strcmp(name, "HOME") == 0)          return KEY_HOME;
    if (strcmp(name, "END") == 0)           return KEY_END;
    if (strcmp(name, "PAGEUP") == 0)        return KEY_PAGEUP;
    if (strcmp(name, "PAGEDOWN") == 0)      return KEY_PAGEDOWN;
    if (strcmp(name, "UP") == 0 || strcmp(name, "UPARROW") == 0)    return KEY_UP;
    if (strcmp(name, "DOWN") == 0 || strcmp(name, "DOWNARROW") == 0) return KEY_DOWN;
    if (strcmp(name, "LEFT") == 0 || strcmp(name, "LEFTARROW") == 0) return KEY_LEFT;
    if (strcmp(name, "RIGHT") == 0 || strcmp(name, "RIGHTARROW") == 0) return KEY_RIGHT;
    if (strcmp(name, "CAPSLOCK") == 0)      return KEY_CAPSLOCK;
    if (strcmp(name, "NUMLOCK") == 0)       return KEY_NUMLOCK;
    if (strcmp(name, "SCROLLLOCK") == 0)    return KEY_SCROLLLOCK;
    if (strcmp(name, "PRINTSCREEN") == 0)   return KEY_PRINTSCREEN;
    if (strcmp(name, "PAUSE") == 0 || strcmp(name, "BREAK") == 0)   return KEY_PAUSE;
    if (strcmp(name, "MENU") == 0 || strcmp(name, "APP") == 0)      return KEY_MENU;
    if (strcmp(name, "F1") == 0)            return KEY_F1;
    if (strcmp(name, "F2") == 0)            return KEY_F2;
    if (strcmp(name, "F3") == 0)            return KEY_F3;
    if (strcmp(name, "F4") == 0)            return KEY_F4;
    if (strcmp(name, "F5") == 0)            return KEY_F5;
    if (strcmp(name, "F6") == 0)            return KEY_F6;
    if (strcmp(name, "F7") == 0)            return KEY_F7;
    if (strcmp(name, "F8") == 0)            return KEY_F8;
    if (strcmp(name, "F9") == 0)            return KEY_F9;
    if (strcmp(name, "F10") == 0)           return KEY_F10;
    if (strcmp(name, "F11") == 0)           return KEY_F11;
    if (strcmp(name, "F12") == 0)           return KEY_F12;

    /* Single printable character */
    if (strlen(name) == 1 && name[0] >= 0x20 && name[0] <= 0x7E)
    {
        return ascii_to_hid[name[0] - 0x20].keycode;
    }

    return KEY_NONE;
}


/*============================================================================*/
/**
  * @brief  Parse a modifier keyword and return its HID modifier bit
  */
/*============================================================================*/
static uint8_t badbt_parse_modifier(const char *name, const char **remainder)
{
    *remainder = NULL;

    struct {
        const char *keyword;
        uint8_t     mod;
    } modifiers[] = {
        {"CTRL",    HID_MOD_LCTRL},
        {"CONTROL", HID_MOD_LCTRL},
        {"ALT",     HID_MOD_LALT},
        {"SHIFT",   HID_MOD_LSHIFT},
        {"GUI",     HID_MOD_LGUI},
        {"WINDOWS", HID_MOD_LGUI},
        {"COMMAND", HID_MOD_LGUI},
    };

    for (int i = 0; i < (int)(sizeof(modifiers) / sizeof(modifiers[0])); i++)
    {
        size_t klen = strlen(modifiers[i].keyword);
        if (strncmp(name, modifiers[i].keyword, klen) == 0)
        {
            char sep = name[klen];
            if (sep == ' ' || sep == '-' || sep == '+' || sep == '\0')
            {
                if (sep != '\0')
                    *remainder = name + klen + 1;
                else
                    *remainder = NULL;
                return modifiers[i].mod;
            }
        }
    }

    return 0;
}


/*============================================================================*/
/**
  * @brief  Parse and execute a single DuckyScript line (BLE HID transport)
  * @retval true if line was valid, false on error
  */
/*============================================================================*/
static bool badbt_parse_line(const char *line)
{
    /* Skip leading whitespace */
    while (*line == ' ' || *line == '\t') line++;

    /* Skip empty lines */
    if (*line == '\0' || *line == '\n' || *line == '\r')
        return true;

    /* REM — comment */
    if (strncmp(line, "REM ", 4) == 0 || strncmp(line, "REM\r", 4) == 0 ||
        strncmp(line, "REM\n", 4) == 0 || strcmp(line, "REM") == 0)
        return true;

    /* DELAY <ms> — broken into 100ms chunks for abort responsiveness */
    if (strncmp(line, "DELAY ", 6) == 0)
    {
        uint32_t ms = (uint32_t)atoi(line + 6);
        while (ms > 0 && badbt_state.running)
        {
            uint32_t chunk = (ms > 100) ? 100 : ms;
            osDelay(chunk);
            ms -= chunk;
            if (badbt_check_abort())
            {
                badbt_state.running = 0;
                break;
            }
        }
        return true;
    }

    /* DEFAULT_DELAY <ms> */
    if (strncmp(line, "DEFAULT_DELAY ", 14) == 0 ||
        strncmp(line, "DEFAULTDELAY ", 13) == 0)
    {
        const char *p = line + (line[7] == '_' ? 14 : 13);
        badbt_state.default_delay_ms = (uint16_t)atoi(p);
        return true;
    }

    /* STRING <text> */
    if (strncmp(line, "STRING ", 7) == 0)
    {
        badbt_type_string(line + 7);
        return true;
    }

    /* REPEAT <n> */
    if (strncmp(line, "REPEAT ", 7) == 0)
    {
        int count = atoi(line + 7);
        if (count > 0 && badbt_state.last_line[0] != '\0')
        {
            for (int i = 0; i < count && badbt_state.running; i++)
            {
                badbt_parse_line(badbt_state.last_line);
                if (badbt_state.default_delay_ms > 0)
                    osDelay(badbt_state.default_delay_ms);
            }
        }
        return true;
    }

    /* Try as modifier combo: CTRL x, GUI r, ALT F4, SHIFT INSERT, etc. */
    {
        uint8_t mod_accum = 0;
        const char *cur = line;
        const char *rest = NULL;

        while (cur && *cur)
        {
            uint8_t m = badbt_parse_modifier(cur, &rest);
            if (m == 0)
                break;
            mod_accum |= m;
            cur = rest;
        }

        if (mod_accum != 0)
        {
            if (cur != NULL && *cur != '\0')
            {
                char keybuf[32];
                strncpy(keybuf, cur, sizeof(keybuf) - 1);
                keybuf[sizeof(keybuf) - 1] = '\0';
                int kl = (int)strlen(keybuf);
                while (kl > 0 && (keybuf[kl-1] == '\r' || keybuf[kl-1] == '\n' ||
                       keybuf[kl-1] == ' '))
                    keybuf[--kl] = '\0';

                uint8_t keycode = badbt_parse_key_name(keybuf);
                if (keycode != KEY_NONE)
                {
                    if (strlen(keybuf) == 1 && keybuf[0] >= 0x20 && keybuf[0] <= 0x7E)
                    {
                        uint8_t char_mod = ascii_to_hid[keybuf[0] - 0x20].shift ?
                                           HID_MOD_LSHIFT : 0;
                        mod_accum |= char_mod;
                    }
                    badbt_send_key(mod_accum, keycode);
                }
                else
                {
                    badbt_send_key(mod_accum, KEY_NONE);
                }
            }
            else
            {
                badbt_send_key(mod_accum, KEY_NONE);
            }
            return true;
        }
    }

    /* Try as standalone key name */
    {
        char keybuf[32];
        strncpy(keybuf, line, sizeof(keybuf) - 1);
        keybuf[sizeof(keybuf) - 1] = '\0';
        int kl = (int)strlen(keybuf);
        while (kl > 0 && (keybuf[kl-1] == '\r' || keybuf[kl-1] == '\n' ||
               keybuf[kl-1] == ' '))
            keybuf[--kl] = '\0';

        uint8_t keycode = badbt_parse_key_name(keybuf);
        if (keycode != KEY_NONE)
        {
            badbt_send_key(0, keycode);
            return true;
        }
    }

    M1_LOG_W(M1_LOGDB_TAG, "Unknown cmd: %s\r\n", line);
    return true;
}


/*============================================================================*/
/**
  * @brief  Count lines in a text buffer
  */
/*============================================================================*/
static uint16_t badbt_count_lines(const char *buf, uint32_t len)
{
    uint16_t count = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        if (buf[i] == '\n')
            count++;
    }
    if (len > 0 && buf[len - 1] != '\n')
        count++;
    return count;
}


/*============================================================================*/
/**
  * @brief  Show execution progress on LCD
  */
/*============================================================================*/
static void badbt_show_progress(const char *filename)
{
    char buf[32];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

    m1_draw_text(&m1_u8g2, 2, 10, 124, "Bad-BT", TEXT_ALIGN_CENTER);

    /* Show filename (truncated) */
    char name_buf[24];
    strncpy(name_buf, filename, sizeof(name_buf) - 1);
    name_buf[sizeof(name_buf) - 1] = '\0';
    m1_draw_text(&m1_u8g2, 2, 24, 124, name_buf, TEXT_ALIGN_CENTER);

    /* Progress */
    snprintf(buf, sizeof(buf), "Line %d / %d",
             badbt_state.current_line, badbt_state.total_lines);
    m1_draw_text(&m1_u8g2, 2, 38, 124, buf, TEXT_ALIGN_CENTER);

    if (badbt_state.running)
        m1_draw_text(&m1_u8g2, 2, 52, 124, "Running...", TEXT_ALIGN_CENTER);
    else
        m1_draw_text(&m1_u8g2, 2, 52, 124, "Done", TEXT_ALIGN_CENTER);

    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Stop", "OK", arrowright_8x8);

    m1_u8g2_nextpage();
}


/*============================================================================*/
/**
  * @brief  Check if user pressed BACK to abort
  * @retval true if abort requested
  */
/*============================================================================*/
static bool badbt_check_abort(void)
{
    S_M1_Main_Q_t q_item;
    S_M1_Buttons_Status btn;

    if (xQueueReceive(main_q_hdl, &q_item, 0) == pdTRUE)
    {
        if (q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            if (xQueueReceive(button_events_q_hdl, &btn, 0) == pdTRUE)
            {
                if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
                 || btn.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK
                 || btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
                 || btn.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


/*============================================================================*/
/**
  * @brief  Show "waiting for connection" screen with abort check
  * @retval true if connected, false if aborted or timeout
  */
/*============================================================================*/
static bool badbt_wait_for_connection(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 10, 124, "Bad-BT", TEXT_ALIGN_CENTER);
    m1_draw_text(&m1_u8g2, 2, 26, 124, m1_badbt_name, TEXT_ALIGN_CENTER);
    m1_draw_text(&m1_u8g2, 2, 42, 124, "Connecting...", TEXT_ALIGN_CENTER);
    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "OK", arrowright_8x8);
    m1_u8g2_nextpage();

    /* Poll for connection with user abort check */
    uint32_t t0 = osKernelGetTickCount();
    uint8_t dots = 0;

    while ((osKernelGetTickCount() - t0) < (BADBT_CONNECT_TIMEOUT * 1000))
    {
        /* Check for connection via ESP32 (1-second polls) */
        uint8_t ret = ble_hid_wait_connect(&badbt_req, 1);
        if (ret == SUCCESS)
        {
            badbt_state.connected = 1;
            return true;
        }

        /* Check for user abort */
        if (badbt_check_abort())
            return false;

        /* Update screen with animated dots */
        dots = (dots + 1) % 4;
        char dot_str[8];
        memset(dot_str, '.', dots);
        dot_str[dots] = '\0';

        m1_u8g2_firstpage();
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 10, 124, "Bad-BT", TEXT_ALIGN_CENTER);
        m1_draw_text(&m1_u8g2, 2, 24, 124, m1_badbt_name, TEXT_ALIGN_CENTER);

        char wait_str[24];
        snprintf(wait_str, sizeof(wait_str), "Pair from target%s", dot_str);
        m1_draw_text(&m1_u8g2, 2, 38, 124, wait_str, TEXT_ALIGN_CENTER);

        /* Show elapsed time */
        uint32_t elapsed = (osKernelGetTickCount() - t0) / 1000;
        char time_str[16];
        snprintf(time_str, sizeof(time_str), "%lus / %ds", (unsigned long)elapsed, BADBT_CONNECT_TIMEOUT);
        m1_draw_text(&m1_u8g2, 2, 50, 124, time_str, TEXT_ALIGN_CENTER);

        m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "OK", arrowright_8x8);
        m1_u8g2_nextpage();
    }

    return false;
}


/*============================================================================*/
/**
  * @brief  Execute a DuckyScript file over BLE HID (assumes already connected)
  * @param  filepath: full path to script on SD card
  * @retval true on success, false on error/abort
  */
/*============================================================================*/
static bool badbt_execute_file(const char *filepath)
{
    FIL fp;
    FRESULT fres;
    UINT bytes_read;
    char script_buf[BADBT_MAX_SCRIPT_SIZE];
    char line_buf[BADBT_MAX_LINE_LEN];

    /* Read script file */
    fres = f_open(&fp, filepath, FA_READ);
    if (fres != FR_OK)
    {
        M1_LOG_E(M1_LOGDB_TAG, "Failed to open: %s (err=%d)\r\n", filepath, fres);
        return false;
    }

    fres = f_read(&fp, script_buf, BADBT_MAX_SCRIPT_SIZE - 1, &bytes_read);
    f_close(&fp);

    if (fres != FR_OK || bytes_read == 0)
    {
        M1_LOG_E(M1_LOGDB_TAG, "Read failed (err=%d, bytes=%lu)\r\n", fres, (unsigned long)bytes_read);
        return false;
    }
    script_buf[bytes_read] = '\0';

    /* Init execution state */
    memset(&badbt_state, 0, sizeof(badbt_state));
    badbt_state.running   = 1;
    badbt_state.connected = 1;
    badbt_state.total_lines = badbt_count_lines(script_buf, bytes_read);

    const char *fname = strrchr(filepath, '/');
    if (fname) fname++; else fname = filepath;

    /* Execute script lines */
    const char *p = script_buf;
    const char *end = script_buf + bytes_read;

    while (p < end && badbt_state.running)
    {
        const char *line_end = p;
        while (line_end < end && *line_end != '\n')
            line_end++;

        size_t line_len = (size_t)(line_end - p);
        if (line_len >= BADBT_MAX_LINE_LEN)
            line_len = BADBT_MAX_LINE_LEN - 1;

        memcpy(line_buf, p, line_len);
        line_buf[line_len] = '\0';

        /* Trim trailing CR */
        if (line_len > 0 && line_buf[line_len - 1] == '\r')
            line_buf[line_len - 1] = '\0';

        badbt_state.current_line++;

        /* Execute the line */
        badbt_parse_line(line_buf);

        /* Save for REPEAT */
        if (strncmp(line_buf, "REPEAT ", 7) != 0)
        {
            strncpy(badbt_state.last_line, line_buf,
                    sizeof(badbt_state.last_line) - 1);
            badbt_state.last_line[sizeof(badbt_state.last_line) - 1] = '\0';
        }

        /* Default delay between commands */
        if (badbt_state.default_delay_ms > 0)
            osDelay(badbt_state.default_delay_ms);

        /* Update progress display every 4 lines */
        if ((badbt_state.current_line & 3) == 0)
            badbt_show_progress(fname);

        /* Check for abort */
        if (badbt_check_abort())
        {
            badbt_state.running = 0;
            break;
        }

        /* Advance past newline */
        p = line_end;
        if (p < end && *p == '\n')
            p++;
    }

    /* Ensure all keys released */
    badbt_release_all();

    /* Show final progress */
    badbt_state.running = 0;
    badbt_show_progress(fname);
    osDelay(500);

    return true;
}


/*============================================================================*/
/**
  * @brief  (Re)init file browser pointed directly at the BadUSB folder.
  *         Deinit first if needed, then init fresh at BADBT_DIR.
  */
/*============================================================================*/
static void badbt_init_file_browser(void)
{
    S_M1_file_browser_hdl *fb = m1_fb_init(&m1_u8g2);

    /* Point directly at BadUSB folder instead of root */
    free(fb->info.dir_name);
    fb->info.dir_name = malloc(strlen(BADBT_DIR) + 1);
    strcpy(fb->info.dir_name, BADBT_DIR);
    fb->dir_level = 1;
    fb->listing_index_buffer = realloc(fb->listing_index_buffer, 2 * sizeof(uint16_t));
    fb->row_index_buffer = realloc(fb->row_index_buffer, 2 * sizeof(uint16_t));
    fb->listing_index_buffer[0] = 0;
    fb->row_index_buffer[0] = 0;
    fb->listing_index_buffer[1] = 0;
    fb->row_index_buffer[1] = 0;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
    m1_u8g2_nextpage();

    m1_fb_display(NULL);
}


/*============================================================================*/
/**
  * @brief  Bad-BT menu entry point
  *
  * Flow: Init HID → Wait for BLE connection → File browser loop
  *       (select & execute payloads, stay connected) → Back = disconnect
  */
/*============================================================================*/
void badbt_run(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    S_M1_file_info *f_info;
    BaseType_t ret;
    char filepath[128];

    /* ---- Phase 1: Init ESP32 & BLE HID ---- */

    {
        ctrl_cmd_t tmp = CTRL_CMD_DEFAULT_REQ();
        badbt_req = tmp;
    }

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 10, 124, "Bad-BT", TEXT_ALIGN_CENTER);
    m1_draw_text(&m1_u8g2, 2, 30, 124, "Init BLE HID...", TEXT_ALIGN_CENTER);
    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "OK", arrowright_8x8);
    m1_u8g2_nextpage();

    /* Ensure ESP32 hardware and SPI task are initialized */
    if ( !m1_esp32_get_init_status() )
        m1_esp32_init();

    if ( !get_esp32_main_init_status() )
        esp32_main_init();

    if ( !get_esp32_main_init_status() )
    {
        m1_message_box(&m1_u8g2, "Bad-BT", "ESP32 not", "ready", " OK ");
        return;
    }

    uint8_t init_ret = ble_hid_init(&badbt_req, m1_badbt_name);
    if (init_ret != SUCCESS)
    {
        char step_msg[16];
        snprintf(step_msg, sizeof(step_msg), "fail step %d", init_ret);
        m1_message_box(&m1_u8g2, "Bad-BT", "BLE HID init", step_msg, " OK ");
        return;
    }

    /* ---- Phase 2: Wait for target device to pair ---- */

    memset(&badbt_state, 0, sizeof(badbt_state));

    if (!badbt_wait_for_connection())
    {
        ble_hid_deinit(&badbt_req);
        if (!badbt_state.connected)
            m1_message_box(&m1_u8g2, "Bad-BT", "Connection", "timeout", " OK ");
        return;
    }

    /* Brief settle delay after connection */
    osDelay(1000);

    /* ---- Phase 3: Connected — file browser loop ---- */

    /* Check SD card */
    if (m1_sdcard_get_status() != SD_access_OK)
    {
        ble_hid_deinit(&badbt_req);
        m1_message_box(&m1_u8g2, "Bad-BT", "SD card not", "available", " OK ");
        return;
    }

    /* Ensure BadUSB directory exists (shared with BadUSB) */
    if (!m1_fb_check_existence(BADBT_DIR))
        m1_fb_make_dir(BADBT_DIR);

    /* Init file browser and navigate directly to BadUSB folder */
    badbt_init_file_browser();

    /* File browser event loop — stays connected until BACK */
    while (1)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret != pdTRUE)
            continue;

        if (q_item.q_evt_type != Q_EVENT_KEYPAD)
            continue;

        ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
        if (ret != pdTRUE)
            continue;

        /* BACK/LEFT = disconnect & exit */
        if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
         || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            m1_fb_deinit();
            break;
        }

        /* Pass button events to file browser */
        f_info = m1_fb_display(&this_button_status);
        if (f_info->status == FB_OK && f_info->file_is_selected)
        {
            /* Build full file path */
            snprintf(filepath, sizeof(filepath), "%s/%s",
                     f_info->dir_name, f_info->file_name);

            /* Show confirmation */
            char msg_line[28];
            strncpy(msg_line, f_info->file_name, sizeof(msg_line) - 1);
            msg_line[sizeof(msg_line) - 1] = '\0';

            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 2, 10, 124, "Run script?", TEXT_ALIGN_CENTER);
            m1_draw_text(&m1_u8g2, 2, 28, 124, msg_line, TEXT_ALIGN_CENTER);
            m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Run", arrowright_8x8);
            m1_u8g2_nextpage();

            /* Wait for OK or BACK */
            bool confirmed = false;
            while (1)
            {
                ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
                if (ret != pdTRUE) continue;
                if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;

                ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
                if (ret != pdTRUE) continue;

                if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK ||
                    this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
                {
                    confirmed = true;
                    break;
                }
                if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK ||
                    this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
                {
                    break;
                }
            }

            if (confirmed)
            {
                bool ok = badbt_execute_file(filepath);
                if (ok)
                    m1_message_box(&m1_u8g2, "Bad-BT", "Script", "complete", " OK ");
                else
                    m1_message_box(&m1_u8g2, "Bad-BT", "Script", "error", " OK ");
            }

            /* Return to BadUSB folder — full reinit for clean state */
            m1_fb_deinit();
            badbt_init_file_browser();
        }
    }

    /* ---- Phase 4: Disconnect & cleanup ---- */
    ble_hid_deinit(&badbt_req);
}

#endif /* M1_APP_BADBT_ENABLE */
