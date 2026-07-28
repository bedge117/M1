/* See COPYING.txt for license details. */

/*
*
*  m1_gpio.h
*
*  M1 GPIO functions
*
* M1 Project
*
*/

#ifndef M1_GPIO_H_
#define M1_GPIO_H_

#define M1_EXT_GPIO_LIST_N						15 // Exclude PA9 (UART1_TX), PA10 (UART1_RX)
													// (PA14 (SWCLK) and PA13 (SWDIO))
#define M1_EXT_GPIO_FIRST_ID					3 // First GPIO pin id in the GPIO buffer

extern S_GPIO_IO_t m1_ext_gpio[];

void menu_gpio_init(void);
void menu_gpio_exit(void);

void gpio_manual_control(void);
void gpio_3_3v_on_gpio(void);
void gpio_5v_on_gpio(void);
void gpio_usb_uart_bridge(void);
void tool_i2c_scanner(void);   /* bit-bang I2C scanner on header pins 6/7 */
uint8_t m1_i2c_scan(uint8_t *out, uint8_t max);  /* headless scan for RPC — returns count */
void tool_pin_reader(void);    /* live logic-level reader for header GPIOs */
void tool_peer_recover(void);  /* bit-bang SWD host to read/recover a peer M1 via header pins */

void ext_power_5V_set(uint8_t set_mode);
void ext_power_3V_set(uint8_t set_mode);

/* App-facing external-header GPIO control (0-based over the user signal pins). */
uint8_t     m1_gpio_ext_app_count(void);
const char *m1_gpio_ext_app_name(uint8_t app_id);
void        m1_gpio_ext_app_mode(uint8_t app_id, uint8_t mode);   /* 0=input, 1=output */
void        m1_gpio_ext_app_write(uint8_t app_id, uint8_t on);
uint8_t     m1_gpio_ext_app_read(uint8_t app_id);
void        m1_gpio_ext_app_release(void);

/* Generic 1-Wire bit-bang primitive on a header pin (for apps: DS18B20, etc.) */
void        m1_ow_init(uint8_t app_id);
int         m1_ow_reset(void);
void        m1_ow_write_byte(uint8_t v);
uint8_t     m1_ow_read_byte(void);
void        m1_ow_deinit(void);
void gpio_gui_update(const S_M1_Menu_t *phmenu, uint8_t sel_item);
void gpio_xkey_handler(S_M1_Key_Event event, uint8_t button_id, uint8_t sel_item);

#endif /* M1_GPIO_H_ */
