/* See COPYING.txt for license details. */

/*
 * m1_badbt.h
 *
 * Bad-BT — BLE HID keyboard injection with DuckyScript parser.
 * Wireless equivalent of BadUSB, sends keystrokes over Bluetooth.
 */

#ifndef M1_BADBT_H_
#define M1_BADBT_H_

#include <stdint.h>
#include <stdbool.h>

/* Bad-BT reuses BadUSB script directory and limits */
#define BADBT_MAX_SCRIPT_SIZE    4096
#define BADBT_MAX_LINE_LEN       256
#define BADBT_DIR                "0:/BadUSB"

/* Bad-BT execution state */
typedef struct
{
    volatile uint8_t  running;
    volatile uint8_t  connected;
    uint16_t current_line;
    uint16_t total_lines;
    uint16_t default_delay_ms;
    char     last_line[BADBT_MAX_LINE_LEN];
} badbt_state_t;

/* Menu entry point */
void badbt_run(void);

#endif /* M1_BADBT_H_ */
