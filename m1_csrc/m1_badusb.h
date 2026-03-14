/* See COPYING.txt for license details. */

/*
 * m1_badusb.h
 *
 * BadUSB — USB HID keyboard injection with DuckyScript parser
 */

#ifndef M1_BADUSB_H_
#define M1_BADUSB_H_

#include <stdint.h>
#include <stdbool.h>

/* BadUSB script max sizes */
#define BADUSB_MAX_SCRIPT_SIZE    4096
#define BADUSB_MAX_LINE_LEN       256
#define BADUSB_DIR                "0:/BadUSB"

/* BadUSB execution state */
typedef struct
{
    volatile uint8_t  running;
    uint16_t current_line;
    uint16_t total_lines;
    uint16_t default_delay_ms;
    char     last_line[BADUSB_MAX_LINE_LEN];
} badusb_state_t;

/* Menu entry point */
void badusb_run(void);

/* Script execution API */
bool badusb_execute_file(const char *filepath);
void badusb_stop(void);

#endif /* M1_BADUSB_H_ */
