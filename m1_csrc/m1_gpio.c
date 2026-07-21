/* See COPYING.txt for license details. */

/*
*
*  m1_gpio.c
*
*  M1 GPIO functions
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_gpio.h"
#include "m1_usb_cdc_msc.h"   /* USB-UART bridge baud control (USART1 on header) */
#include "m1_log_debug.h"     /* M1_LOG_I — log I2C scan results over USB */

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG	"GPIO"

#define THIS_LCD_MENU_TEXT_FIRST_ROW_Y			11
#define THIS_LCD_MENU_TEXT_FRAME_FIRST_ROW_Y	1
#define THIS_LCD_MENU_TEXT_ROW_SPACE			10

//************************** C O N S T A N T **********************************/

const char *m1_ext_gpio_label[M1_EXT_GPIO_LIST_N] = {	"Power 3.3V",
														"Power 5.0V",
														"",
														"Pin PE2",
														"Pin PE4",
														"Pin PE5",
														"Pin PE6",
														"Pin PD12",
														"Pin PD13",
														"Pin PA14",
														"Pin PA13",
														/*"Pin PA9",*/
														/*"Pin PA10",*/
														"Pin PC2",
														"Pin PC3",
														"Pin PD0",
														"Pin PD1"
													};

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

S_GPIO_IO_t m1_ext_gpio[M1_EXT_GPIO_LIST_N] = {	{.gpio_port = EN_EXT_3V3_GPIO_Port, .gpio_pin = EN_EXT_3V3_Pin},
												{.gpio_port = EN_EXT_5V_GPIO_Port, .gpio_pin = EN_EXT_5V_Pin},
												{.gpio_port = EN_EXT2_5V_GPIO_Port, .gpio_pin = EN_EXT2_5V_Pin},
												{.gpio_port = PE2_GPIO_Port, .gpio_pin = PE2_Pin},
												{.gpio_port = PE2_GPIO_Port, .gpio_pin = PE4_Pin},
												{.gpio_port = PE2_GPIO_Port, .gpio_pin = PE5_Pin},
												{.gpio_port = PE2_GPIO_Port, .gpio_pin = PE6_Pin},
												{.gpio_port = PD12_GPIO_Port, .gpio_pin = PD12_Pin},
												{.gpio_port = PD13_GPIO_Port, .gpio_pin = PD13_Pin},
												{.gpio_port = SWCLK_GPIO_Port, .gpio_pin = SWCLK_Pin},
												{.gpio_port = SWDIO_GPIO_Port, .gpio_pin = SWDIO_Pin},
												/*{.gpio_port = UART_1_TX_GPIO_Port, .gpio_pin = UART_1_TX_Pin},*/
												/*{.gpio_port = UART_1_RX_GPIO_Port, .gpio_pin = UART_1_RX_Pin},*/
												{.gpio_port = PC2_GPIO_Port, .gpio_pin = PC2_Pin},
												{.gpio_port = PC3_GPIO_Port, .gpio_pin = PC3_Pin},
												{.gpio_port = PD0_GPIO_Port, .gpio_pin = PD0_Pin},
												{.gpio_port = PD1_GPIO_Port, .gpio_pin = PD1_Pin}
											};

static uint8_t m1_ext_gpio_stat[M1_EXT_GPIO_LIST_N] = {0};
static uint8_t m1_ext_gpio_id = M1_EXT_GPIO_FIRST_ID; // Default to the first ext. GPIO [PE2_GPIO_Port, PE2_Pin]

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_gpio_init(void);
void menu_gpio_exit(void);

void gpio_manual_control(void);
void gpio_5v_on_gpio(void);
void gpio_3_3v_on_gpio(void);
void gpio_usb_uart_bridge(void);
void ext_power_5V_set(uint8_t set_mode);
void ext_power_3V_set(uint8_t set_mode);
void gpio_gui_update(const S_M1_Menu_t *phmenu, uint8_t sel_item);
void gpio_xkey_handler(S_M1_Key_Event event, uint8_t button_id, uint8_t sel_item);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/******************************************************************************/
/**
  * @brief Initializes display for this sub-menu item.
  * @param
  * @retval
  */
/******************************************************************************/
void menu_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    for(i=0; i<M1_EXT_GPIO_LIST_N; i++)
    {
    	if ( i >= M1_EXT_GPIO_FIRST_ID ) // Do not reinitialize power control pins
    	{
    		GPIO_InitStruct.Pin = m1_ext_gpio[i].gpio_pin;
    		HAL_GPIO_Init(m1_ext_gpio[i].gpio_port, &GPIO_InitStruct);
    	}
    	HAL_GPIO_WritePin(m1_ext_gpio[i].gpio_port, m1_ext_gpio[i].gpio_pin, GPIO_PIN_RESET);
    	m1_ext_gpio_stat[i] = 0;
    }

    m1_ext_gpio_id = M1_EXT_GPIO_FIRST_ID; // Default to the first ext. GPIO [PE2_GPIO_Port, PE2_Pin]
} // void menu_gpio_init(void)


/******************************************************************************/
/**
  * @brief Exits this sub-menu and return to the upper level menu.
  * @param
  * @retval
  */
/******************************************************************************/
void menu_gpio_exit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    for(i=M1_EXT_GPIO_FIRST_ID; i<M1_EXT_GPIO_LIST_N; i++)
    {
    	GPIO_InitStruct.Pin = m1_ext_gpio[i].gpio_pin;
    	HAL_GPIO_Init(m1_ext_gpio[i].gpio_port, &GPIO_InitStruct);
    }

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Pulldown for SWCLK
    GPIO_InitStruct.Pin = SWCLK_Pin;
    HAL_GPIO_Init(SWCLK_GPIO_Port, &GPIO_InitStruct); // SWCLK
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Pullup for SWDIO
    GPIO_InitStruct.Pin = SWDIO_Pin;
    HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct); // SWDIO

    for(i=0; i<M1_EXT_GPIO_FIRST_ID; i++) // Reset power control pins
    {
    	HAL_GPIO_WritePin(m1_ext_gpio[i].gpio_port, m1_ext_gpio[i].gpio_pin, GPIO_PIN_RESET);
    }
} // void menu_gpio_exit(void)



/******************************************************************************/
/**
  * @brief
  * @param
  * @retval
  */
/******************************************************************************/
void gpio_manual_control(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t prn_name[GUI_DISP_LINE_LEN_MAX + 1] = {0};

    m1_ext_gpio_stat[m1_ext_gpio_id] ^= 1; // Toggle

    HAL_GPIO_WritePin(m1_ext_gpio[m1_ext_gpio_id].gpio_port, m1_ext_gpio[m1_ext_gpio_id].gpio_pin, m1_ext_gpio_stat[m1_ext_gpio_id]);

	sprintf(prn_name, "%s: %s", m1_ext_gpio_label[m1_ext_gpio_id], (m1_ext_gpio_stat[m1_ext_gpio_id]==1)?"ON":"OFF");
	m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
	u8g2_NextPage(&m1_u8g2); // Update display RAM

	xQueueReset(main_q_hdl); // Reset main q before return
} // void gpio_manual_control(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void gpio_3_3v_on_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t prn_name[GUI_DISP_LINE_LEN_MAX + 1] = {0};

    m1_ext_gpio_stat[0] ^= 1; // Toggle
    if ( m1_ext_gpio_stat[0] )
    {
    	m1_ext_gpio_stat[1] = 0; // 3.3V and 5.0V must not be turned ON at the same time
        HAL_GPIO_WritePin(m1_ext_gpio[1].gpio_port, m1_ext_gpio[1].gpio_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m1_ext_gpio[2].gpio_port, m1_ext_gpio[2].gpio_pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(m1_ext_gpio[0].gpio_port, m1_ext_gpio[0].gpio_pin, m1_ext_gpio_stat[0]);

	sprintf(prn_name, "%s: %s", m1_ext_gpio_label[0], (m1_ext_gpio_stat[0]==1)?"ON":"OFF");
	m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
	u8g2_NextPage(&m1_u8g2); // Update display RAM

	xQueueReset(main_q_hdl); // Reset main q before return
} // void gpio_3_3v_on_gpio(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void gpio_5v_on_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t prn_name[GUI_DISP_LINE_LEN_MAX + 1] = {0};

    m1_ext_gpio_stat[1] ^= 1; // Toggle
    if ( m1_ext_gpio_stat[1] )
    {
    	m1_ext_gpio_stat[0] = 0; // 3.3V and 5.0V must not be turned ON at the same time
    	HAL_GPIO_WritePin(m1_ext_gpio[0].gpio_port, m1_ext_gpio[0].gpio_pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(m1_ext_gpio[1].gpio_port, m1_ext_gpio[1].gpio_pin, m1_ext_gpio_stat[1]);
    HAL_GPIO_WritePin(m1_ext_gpio[2].gpio_port, m1_ext_gpio[2].gpio_pin, m1_ext_gpio_stat[1]);

	sprintf(prn_name, "%s: %s", m1_ext_gpio_label[1], (m1_ext_gpio_stat[1]==1)?"ON":"OFF");
	m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
	u8g2_NextPage(&m1_u8g2); // Update display RAM

	xQueueReset(main_q_hdl); // Reset main q before return
} // void gpio_5v_on_gpio(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
/*
 * USB-UART bridge: the M1 acts as a USB<->serial adapter. The USB CDC is bridged
 * to USART1 on the external header (Pin 12 = PA9 TX, Pin 13 = PA10 RX). The data
 * path runs continuously in vUsb2SerTask/vSer2UsbTask — any non-RPC bytes on the
 * USB CDC are forwarded to USART1 and vice versa. This screen lets you pick the
 * line rate on-device (a host terminal can also set it via the COM port).
 *
 * NOTE: the header UART is one pin off from the Flipper Zero (Flipper TX/RX are
 * pins 13/14), so Flipper UART hats are NOT drop-in — wire M1 pin 12->hat RX,
 * pin 13->hat TX, share GND/power.
 */
void gpio_usb_uart_bridge(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	static const uint32_t bauds[] = {
		9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
	};
	const int n_bauds = (int)(sizeof(bauds) / sizeof(bauds[0]));
	int sel = 4;        /* default 115200 */
	int applied = -1;
	bool redraw = true;
	char line[24];

	/* Start the selector on the currently active rate if it's in the list. */
	uint32_t cur = m1_usb_uart_get_baud();
	for (int i = 0; i < n_bauds; i++) if (bauds[i] == cur) { sel = i; break; }

	/* Force raw USB<->USART1 forwarding while this screen is open, so the bridge
	 * works even if qMonstatek already latched RPC active (cleared on exit). */
	m1_uart_bridge_active = true;

	/* Apply the default/selected rate on entry so the bridge is live immediately. */
	m1_usb_uart_set_baud(bauds[sel]);
	applied = sel;

	while (1)
	{
		if (redraw)
		{
			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

			u8g2_DrawStr(&m1_u8g2, 2, 10, "USB-UART Bridge");

			snprintf(line, sizeof(line), "Baud: %lu%s",
			         (unsigned long)bauds[sel], (sel == applied) ? "" : " *");
			u8g2_DrawStr(&m1_u8g2, 2, 24, line);

			u8g2_DrawStr(&m1_u8g2, 2, 34, "Pins: TX=12 RX=13");

			if (sel == applied)
				u8g2_DrawStr(&m1_u8g2, 2, 44, "USB<->Serial active");
			else
				u8g2_DrawStr(&m1_u8g2, 2, 44, "Press OK to apply");

			u8g2_DrawStr(&m1_u8g2, 2, 62, "Up/Dn Baud  Back Exit");
			m1_u8g2_nextpage();
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);

			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
			 || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_uart_bridge_active = false;   /* restore RPC/qMonstatek routing */
				xQueueReset(main_q_hdl);
				break;
			}
			else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				sel = (sel > 0) ? sel - 1 : n_bauds - 1;
				redraw = true;
			}
			else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				sel = (sel < n_bauds - 1) ? sel + 1 : 0;
				redraw = true;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
			      || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_usb_uart_set_baud(bauds[sel]);
				applied = sel;
				redraw = true;
			}
		}
	}

} // void gpio_usb_uart_bridge(void)


/*============================================================================*/
/*            H A R D W A R E   T O O L S  (header pins)                       */
/*============================================================================*/
/*
 * Bit-banged I2C scanner on the external header:
 *   Pin 6 = PD12 = SCL,  Pin 7 = PD13 = SDA.
 * Software I2C (open-drain GPIO) — the header pins are plain GPIO (confirmed in
 * the CubeMX .ioc: no I2C4 peripheral), so bit-banging avoids any AF/timing
 * dependence and just works. Internal pull-ups are enabled; typical I2C breakout
 * boards also carry their own. 3.3V bus (header pin 9 for power, pin 8/18 GND).
 */
#define TOOL_SCL_PORT   PD12_GPIO_Port
#define TOOL_SCL_PIN    PD12_Pin
#define TOOL_SDA_PORT   PD13_GPIO_Port
#define TOOL_SDA_PIN    PD13_Pin

static void tool_i2c_delay(void) { for (volatile int i = 0; i < 500; i++) __NOP(); }  /* ~slow, safe */

#define TOOL_SCL_HI()   HAL_GPIO_WritePin(TOOL_SCL_PORT, TOOL_SCL_PIN, GPIO_PIN_SET)
#define TOOL_SCL_LO()   HAL_GPIO_WritePin(TOOL_SCL_PORT, TOOL_SCL_PIN, GPIO_PIN_RESET)
#define TOOL_SDA_HI()   HAL_GPIO_WritePin(TOOL_SDA_PORT, TOOL_SDA_PIN, GPIO_PIN_SET)
#define TOOL_SDA_LO()   HAL_GPIO_WritePin(TOOL_SDA_PORT, TOOL_SDA_PIN, GPIO_PIN_RESET)
#define TOOL_SDA_RD()   HAL_GPIO_ReadPin(TOOL_SDA_PORT, TOOL_SDA_PIN)

static void tool_i2c_pins_init(void)
{
	GPIO_InitTypeDef g = {0};
	g.Mode  = GPIO_MODE_OUTPUT_OD;   /* open-drain: low = drive, high = release (pulled up) */
	g.Pull  = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin = TOOL_SCL_PIN; HAL_GPIO_Init(TOOL_SCL_PORT, &g);
	g.Pin = TOOL_SDA_PIN; HAL_GPIO_Init(TOOL_SDA_PORT, &g);
	TOOL_SCL_HI(); TOOL_SDA_HI();     /* bus idle */
}

static void tool_i2c_pins_release(void)
{
	/* Restore both pins to their power-on state (push-pull output, low). */
	GPIO_InitTypeDef g = {0};
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin = TOOL_SCL_PIN; HAL_GPIO_Init(TOOL_SCL_PORT, &g);
	g.Pin = TOOL_SDA_PIN; HAL_GPIO_Init(TOOL_SDA_PORT, &g);
	TOOL_SCL_LO(); TOOL_SDA_LO();
}

static void tool_i2c_start(void)
{
	TOOL_SDA_HI(); TOOL_SCL_HI(); tool_i2c_delay();
	TOOL_SDA_LO(); tool_i2c_delay();
	TOOL_SCL_LO(); tool_i2c_delay();
}

static void tool_i2c_stop(void)
{
	TOOL_SDA_LO(); tool_i2c_delay();
	TOOL_SCL_HI(); tool_i2c_delay();
	TOOL_SDA_HI(); tool_i2c_delay();
}

/* Write a byte, return the ACK bit (0 = ACK, 1 = NACK). */
static uint8_t tool_i2c_write(uint8_t b)
{
	for (int i = 0; i < 8; i++)
	{
		if (b & 0x80) TOOL_SDA_HI(); else TOOL_SDA_LO();
		b <<= 1;
		tool_i2c_delay();
		TOOL_SCL_HI(); tool_i2c_delay();
		TOOL_SCL_LO(); tool_i2c_delay();
	}
	TOOL_SDA_HI();                    /* release SDA for ACK */
	tool_i2c_delay();
	TOOL_SCL_HI(); tool_i2c_delay();
	uint8_t ack = (TOOL_SDA_RD() == GPIO_PIN_RESET) ? 0 : 1;
	TOOL_SCL_LO(); tool_i2c_delay();
	return ack;
}

/* Probe a 7-bit address: START + addr(write). ACK => a device is present. */
static bool tool_i2c_probe(uint8_t addr7)
{
	tool_i2c_start();
	uint8_t ack = tool_i2c_write((uint8_t)((addr7 << 1) | 0));
	tool_i2c_stop();
	return (ack == 0);
}

void tool_i2c_scanner(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t found[16];
	int nfound = 0;
	bool rescan = true;
	char line[24];

	tool_i2c_pins_init();

	while (1)
	{
		if (rescan)
		{
			rescan = false;
			/* Scan 0x08..0x77. Require TWO consecutive ACKs before reporting an
			 * address — filters transient noise / weak-pull-up false positives. */
			nfound = 0;
			for (uint8_t a = 0x08; a <= 0x77; a++)
			{
				if (tool_i2c_probe(a) && tool_i2c_probe(a) && nfound < (int)sizeof(found))
					found[nfound++] = a;
			}

			/* Log results over USB (readable in a terminal / by qMonstatek). */
			if (nfound == 0)
				M1_LOG_I(M1_LOGDB_TAG, "I2C scan: no devices\r\n");
			for (int i = 0; i < nfound; i++)
				M1_LOG_I(M1_LOGDB_TAG, "I2C found 0x%02X\r\n", found[i]);

			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 2, 9, "I2C Scanner");

			if (nfound == 0)
			{
				u8g2_DrawStr(&m1_u8g2, 2, 26, "No devices found");
				u8g2_DrawStr(&m1_u8g2, 2, 38, "SDA=pin7 SCL=pin6");
			}
			else
			{
				if (nfound > 8) snprintf(line, sizeof(line), "Found %d (8 shown):", nfound);
				else            snprintf(line, sizeof(line), "Found %d:", nfound);
				u8g2_DrawStr(&m1_u8g2, 2, 21, line);

				/* Up to 8 addresses, 4 per row on two rows (y=33, y=45) — kept
				 * clear of the divider/hint at the bottom. */
				int shown = (nfound > 8) ? 8 : nfound;
				char row[24]; int col = 0; uint8_t y = 33;
				row[0] = '\0';
				for (int i = 0; i < shown; i++)
				{
					char a[6]; snprintf(a, sizeof(a), "%02X ", found[i]);
					strncat(row, a, sizeof(row) - strlen(row) - 1);
					if (++col == 4 || i == shown - 1)
					{
						u8g2_DrawStr(&m1_u8g2, 2, y, row);
						y += 12; col = 0; row[0] = '\0';
					}
				}
			}

			/* Divider keeps the list off the button hint. */
			u8g2_DrawHLine(&m1_u8g2, 0, 54, 128);
			u8g2_DrawStr(&m1_u8g2, 2, 63, "OK:Rescan  Back:Exit");
			m1_u8g2_nextpage();
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
			 || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				tool_i2c_pins_release();
				xQueueReset(main_q_hdl);
				break;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
			      || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				rescan = true;
			}
		}
	}
} // void tool_i2c_scanner(void)


/*
 * Header pin logic reader: shows the live HIGH/LOW level of the general-purpose
 * header GPIOs. Pins are set to input while reading and restored to their
 * power-on output-low state on exit. (UART pins 12/13 and SWD pins 10/11 are left
 * alone so the bridge/debug aren't disturbed.)
 */
typedef struct { const char *name; GPIO_TypeDef *port; uint16_t pin; } tool_pin_t;

void tool_pin_reader(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	char line[26];

	static const tool_pin_t pins[] = {
		{ "PE2", PE2_GPIO_Port, PE2_Pin }, { "PE4", PE2_GPIO_Port, PE4_Pin },
		{ "PE5", PE2_GPIO_Port, PE5_Pin }, { "PE6", PE2_GPIO_Port, PE6_Pin },
		{ "PD12", PD12_GPIO_Port, PD12_Pin }, { "PD13", PD13_GPIO_Port, PD13_Pin },
		{ "PC2", PC2_GPIO_Port, PC2_Pin }, { "PC3", PC3_GPIO_Port, PC3_Pin },
		{ "PD0", PD0_GPIO_Port, PD0_Pin }, { "PD1", PD1_GPIO_Port, PD1_Pin },
	};
	const int npins = (int)(sizeof(pins) / sizeof(pins[0]));

	/* Configure all as input for reading. */
	for (int i = 0; i < npins; i++)
	{
		GPIO_InitTypeDef g = {0};
		g.Pin = pins[i].pin; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
		g.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(pins[i].port, &g);
	}

	while (1)
	{
		m1_u8g2_firstpage();
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 2, 10, "Pin Reader (live)");
		/* Two columns of 5. */
		for (int i = 0; i < npins; i++)
		{
			int lvl = (HAL_GPIO_ReadPin(pins[i].port, pins[i].pin) == GPIO_PIN_SET) ? 1 : 0;
			int col = i / 5, row = i % 5;
			snprintf(line, sizeof(line), "%-4s:%s", pins[i].name, lvl ? "HI" : "LO");
			u8g2_DrawStr(&m1_u8g2, 2 + col * 64, 22 + row * 8, line);
		}
		u8g2_DrawStr(&m1_u8g2, 2, 62, "Back Exit");
		m1_u8g2_nextpage();

		/* Refresh ~5x/sec; poll buttons without blocking. */
		if (xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(200)) == pdTRUE
		    && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
			 || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				/* Restore to power-on output-low. */
				for (int i = 0; i < npins; i++)
				{
					GPIO_InitTypeDef g = {0};
					g.Pin = pins[i].pin; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
					g.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(pins[i].port, &g);
					HAL_GPIO_WritePin(pins[i].port, pins[i].pin, GPIO_PIN_RESET);
				}
				xQueueReset(main_q_hdl);
				break;
			}
		}
	}
} // void tool_pin_reader(void)


/******************************************************************************/
/**
  * @brief
  * @param None
  * @retval None
  */
/******************************************************************************/
void ext_power_5V_set(uint8_t set_mode)
{
	HAL_GPIO_WritePin(EN_EXT_5V_GPIO_Port, EN_EXT_5V_Pin, set_mode);
	HAL_GPIO_WritePin(EN_EXT2_5V_GPIO_Port, EN_EXT2_5V_Pin, set_mode);
} // void ext_power_5V_set(uint8_t set_mode)


/******************************************************************************/
/**
  * @brief
  * @param None
  * @retval None
  */
/******************************************************************************/
void ext_power_3V_set(uint8_t set_mode)
{
	  HAL_GPIO_WritePin(EN_EXT_3V3_GPIO_Port, EN_EXT_3V3_Pin, set_mode);
} // void ext_power_5V_set(uint8_t set_mode)



/******************************************************************************/
/**
  * @brief
  * @param None
  * @retval None
  */
/******************************************************************************/
void gpio_gui_update(const S_M1_Menu_t *phmenu, uint8_t sel_item)
{
	uint8_t i, n_items;
	uint8_t menu_text_y;
	uint8_t prn_name[GUI_DISP_LINE_LEN_MAX + 1] = {0};
	uint16_t msg_len, msg_id;

	n_items = phmenu->num_submenu_items;
	menu_text_y = THIS_LCD_MENU_TEXT_FIRST_ROW_Y;

	/* Graphic work starts here */
	m1_u8g2_firstpage(); // This call required for page drawing in mode 1
    do
    {
    	for (i=0; i<n_items; i++)
    	{
    		if ( i==sel_item )
    		{
    			// Draw box for selected menu item with text color
    			u8g2_DrawBox(&m1_u8g2, 0, menu_text_y - THIS_LCD_MENU_TEXT_ROW_SPACE + 2, M1_LCD_SUB_MENU_TEXT_FRAME_W, THIS_LCD_MENU_TEXT_ROW_SPACE);
    			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set to background color
    			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    			u8g2_DrawStr(&m1_u8g2, 4, menu_text_y, phmenu->submenu[i]->title);
    			if ( i==0 ) // Index of GPIO Control
    			{
    		    	// Draw arrows left and right
    		    	u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 40, menu_text_y - THIS_LCD_MENU_TEXT_ROW_SPACE + 2, 10, 10, arrowleft_10x10);
    		    	u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 20, menu_text_y - THIS_LCD_MENU_TEXT_ROW_SPACE + 2, 10, 10, arrowright_10x10);
    			}
    			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // return to text color
    			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N); // return to default font
    		}
    		else
    		{
    			u8g2_DrawStr(&m1_u8g2, 4, menu_text_y, phmenu->submenu[i]->title);
    		}
    		menu_text_y += THIS_LCD_MENU_TEXT_ROW_SPACE;
    	} // for (i=0; i<n_items; i++)

    	// Draw info box at the bottom
    	m1_info_box_display_init(false);

    	switch ( sel_item )
    	{
    		case 0: // GPIO
    			sprintf(prn_name, "%s: %s", m1_ext_gpio_label[m1_ext_gpio_id], (m1_ext_gpio_stat[m1_ext_gpio_id]==1)?"ON":"OFF");
    			m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
    			break;

    		case 1: // Power 3.3V
    			sprintf(prn_name, "%s: %s", m1_ext_gpio_label[0], (m1_ext_gpio_stat[0]==1)?"ON":"OFF");
    	    	m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
    			break;

    		case 2: // Power 5.0V
    			sprintf(prn_name, "%s: %s", m1_ext_gpio_label[1], (m1_ext_gpio_stat[1]==1)?"ON":"OFF");
    	    	m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);
    			break;

    		case 3:
    	    	m1_info_box_display_draw(INFO_BOX_ROW_1, "USB<->UART (hdr 12/13)");
    			break;

    		default: // Unknown selection
    			break;
    	} // switch ( sel_item )
    } while (m1_u8g2_nextpage());

} // void gpio_gui_update(const S_M1_Menu_t *phmenu, uint8_t sel_item)



/******************************************************************************/
/**
  * @brief
  * @param None
  * @retval None
  */
/******************************************************************************/
void gpio_xkey_handler(S_M1_Key_Event event, uint8_t button_id, uint8_t sel_item)
{
	uint8_t prn_name[GUI_DISP_LINE_LEN_MAX + 1] = {0};

	if ( sel_item != 0) // Not the index of GPIO Control
		return;

	if ( event==BUTTON_EVENT_CLICK )
	{
		if ( button_id==BUTTON_LEFT_KP_ID ) // Left arrow key
		{
			m1_ext_gpio_id--;
			if ( m1_ext_gpio_id < M1_EXT_GPIO_FIRST_ID )
				m1_ext_gpio_id = M1_EXT_GPIO_LIST_N - 1;
		} // if ( button_id==BUTTON_LEFT_KP_ID )
		else if ( button_id==BUTTON_RIGHT_KP_ID ) // Right arrow key
		{
			m1_ext_gpio_id++;
			if ( m1_ext_gpio_id >= M1_EXT_GPIO_LIST_N )
				m1_ext_gpio_id = M1_EXT_GPIO_FIRST_ID;
		}

		sprintf(prn_name, "%s: %s", m1_ext_gpio_label[m1_ext_gpio_id], (m1_ext_gpio_stat[m1_ext_gpio_id]==1)?"ON":"OFF");
    	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set to background color
    	// Clear old content
    	m1_info_box_display_clear();
    	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set to text color
		m1_info_box_display_draw(INFO_BOX_ROW_1, prn_name);

		m1_u8g2_nextpage(); // Update LCD display RAM
	} // if ( event==BUTTON_EVENT_CLICK )
} // void gpio_xkey_handler(S_M1_Key_Event event, uint8_t button_id, uint8_t)
