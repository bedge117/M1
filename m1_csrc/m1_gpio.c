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
#include "m1_watchdog.h"      /* m1_wdt_reset — kick IWDG during the long SWD flash */
#include "m1_file_browser.h"  /* SD file picker for Recover Peer (working FW only) */
#include "ff.h"               /* FatFS — read the recovery image from SD (peer recover) */

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



/*============================================================================*/
/*        A P P - F A C I N G   E X T   G P I O   A P I  (0-based ids)         */
/*                                                                            */
/* Lets external .m1app apps drive/read the header GPIO signal pins. App ids  */
/* are 0-based over the user pins only (firmware index = app_id +             */
/* M1_EXT_GPIO_FIRST_ID), so the 3 power-rail control pins are hidden.        */
/* mode()/write()/read() cover both output-drive and input-monitor use.      */
/*============================================================================*/

uint8_t m1_gpio_ext_app_count(void)
{
    return (uint8_t)(M1_EXT_GPIO_LIST_N - M1_EXT_GPIO_FIRST_ID);
}

const char *m1_gpio_ext_app_name(uint8_t app_id)
{
    if (app_id >= m1_gpio_ext_app_count()) return "";
    return m1_ext_gpio_label[app_id + M1_EXT_GPIO_FIRST_ID];
}

/* mode: 0 = input, 1 = push-pull output (driven low on entry). */
void m1_gpio_ext_app_mode(uint8_t app_id, uint8_t mode)
{
    if (app_id >= m1_gpio_ext_app_count()) return;
    uint8_t i = (uint8_t)(app_id + M1_EXT_GPIO_FIRST_ID);

    GPIO_InitTypeDef g = {0};
    g.Pin   = m1_ext_gpio[i].gpio_pin;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Mode  = mode ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    HAL_GPIO_Init(m1_ext_gpio[i].gpio_port, &g);

    if (mode)
    {
        HAL_GPIO_WritePin(m1_ext_gpio[i].gpio_port, m1_ext_gpio[i].gpio_pin, GPIO_PIN_RESET);
        m1_ext_gpio_stat[i] = 0;
    }
}

void m1_gpio_ext_app_write(uint8_t app_id, uint8_t on)
{
    if (app_id >= m1_gpio_ext_app_count()) return;
    uint8_t i = (uint8_t)(app_id + M1_EXT_GPIO_FIRST_ID);

    HAL_GPIO_WritePin(m1_ext_gpio[i].gpio_port, m1_ext_gpio[i].gpio_pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    m1_ext_gpio_stat[i] = on ? 1 : 0;
}

uint8_t m1_gpio_ext_app_read(uint8_t app_id)
{
    if (app_id >= m1_gpio_ext_app_count()) return 0;
    uint8_t i = (uint8_t)(app_id + M1_EXT_GPIO_FIRST_ID);
    return (HAL_GPIO_ReadPin(m1_ext_gpio[i].gpio_port, m1_ext_gpio[i].gpio_pin) == GPIO_PIN_SET) ? 1 : 0;
}

/* Park all user header pins in a safe state (analog + pulldown) and restore
 * the SWD debug pins (PA14/PA13) to their alternate function. Call on exit. */
void m1_gpio_ext_app_release(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i;

    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_PULLDOWN;
    for (i = M1_EXT_GPIO_FIRST_ID; i < M1_EXT_GPIO_LIST_N; i++)
    {
        g.Pin = m1_ext_gpio[i].gpio_pin;
        HAL_GPIO_Init(m1_ext_gpio[i].gpio_port, &g);
    }

    g.Mode      = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF0_SWJ;
    g.Pull      = GPIO_PULLDOWN;
    g.Pin       = SWCLK_Pin;
    HAL_GPIO_Init(SWCLK_GPIO_Port, &g);
    g.Pull      = GPIO_PULLUP;
    g.Pin       = SWDIO_Pin;
    HAL_GPIO_Init(SWDIO_GPIO_Port, &g);
}



/*============================================================================*/
/*        1 - W I R E   B I T - B A N G   P R I M I T I V E   (for apps)       */
/*                                                                            */
/* Microsecond-timed, IRQ-masked reset/read/write on one header pin, so apps  */
/* can talk to 1-Wire devices (DS18B20, iButton, ...). The SDK has no us-      */
/* accurate, IRQ-safe timing of its own, so it must come from here. Uses the  */
/* pin as open-drain; an external 4.7k pull-up to +3.3V is required. Device-   */
/* specific logic (command sequences, CRC, temperature math) stays in the app.*/
/*============================================================================*/

static GPIO_TypeDef *s_ow_port;
static uint16_t      s_ow_pin;

static void ow_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ow_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static inline void    ow_drive_low(void) { s_ow_port->BSRR = (uint32_t)s_ow_pin << 16; }
static inline void    ow_release(void)   { s_ow_port->BSRR = (uint32_t)s_ow_pin; }
static inline uint8_t ow_level(void)     { return (s_ow_port->IDR & s_ow_pin) ? 1 : 0; }

/* Configure header pin `app_id` (0-based over the user pins) as an open-drain
 * 1-Wire line. Needs an external 4.7k pull-up to +3.3V. */
void m1_ow_init(uint8_t app_id)
{
    if (app_id >= m1_gpio_ext_app_count()) return;
    uint8_t i = (uint8_t)(app_id + M1_EXT_GPIO_FIRST_ID);
    s_ow_port = m1_ext_gpio[i].gpio_port;
    s_ow_pin  = m1_ext_gpio[i].gpio_pin;

    GPIO_InitTypeDef g = {0};
    g.Pin   = s_ow_pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;   /* weak internal pull-up: backs up the external 4.7k
                              * and makes a disconnected line read HIGH (idle),
                              * so "no device" is detected instead of faked low */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(s_ow_port, &g);
    ow_release();
    ow_dwt_init();
}

/* Reset pulse + presence detect. Returns 1 if a device responded, else 0. */
int m1_ow_reset(void)
{
    int presence;
    if (!s_ow_port) return 0;
    __disable_irq();
    ow_drive_low(); ow_delay_us(480);
    ow_release();   ow_delay_us(70);
    presence = ow_level() ? 0 : 1;   /* a device pulls the line low */
    __enable_irq();
    ow_delay_us(410);
    return presence;
}

static void ow_write_bit(int bit)
{
    __disable_irq();
    ow_drive_low();
    if (bit) { ow_delay_us(6);  ow_release(); ow_delay_us(64); }
    else     { ow_delay_us(60); ow_release(); ow_delay_us(10); }
    __enable_irq();
}

static int ow_read_bit(void)
{
    int b;
    __disable_irq();
    ow_drive_low(); ow_delay_us(6);
    ow_release();   ow_delay_us(9);
    b = ow_level();
    __enable_irq();
    ow_delay_us(55);
    return b;
}

void m1_ow_write_byte(uint8_t v)
{
    if (!s_ow_port) return;
    for (int i = 0; i < 8; i++) { ow_write_bit(v & 1); v >>= 1; }
}

uint8_t m1_ow_read_byte(void)
{
    uint8_t v = 0;
    if (!s_ow_port) return 0;
    for (int i = 0; i < 8; i++) { v >>= 1; if (ow_read_bit()) v |= 0x80; }
    return v;
}

/* Release the 1-Wire pin to a safe state. */
void m1_ow_deinit(void)
{
    if (!s_ow_port) return;
    GPIO_InitTypeDef g = {0};
    g.Pin  = s_ow_pin;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(s_ow_port, &g);
    s_ow_port = 0;
}



/*============================================================================*/
/*   P E E R   R E C O V E R Y  —  Phase 1: SWD connect + read IDCODE         */
/*                                                                            */
/* Turn this (good) M1 into a bit-bang SWD host (a mini ST-Link) for another  */
/* M1's STM32 debug port. The SW-DP is alive whenever the target core is      */
/* powered — it does NOT depend on the target's firmware — so this recovers a */
/* fully bricked M1 (the interface the user has actually used to recover from */
/* the pins before). Phase 1 only performs the SWD line-reset + JTAG-to-SWD   */
/* handshake and reads DPIDR (the debug-port IDCODE) — ZERO flash writes — to */
/* prove the link before we add halt/erase/program in later phases.           */
/*                                                                            */
/* Wiring (good M1 header -> target M1 header):                               */
/*   good pin 2  (PE2) --> target pin 10 (PA14 SWCLK)                         */
/*   good pin 3  (PE4) --> target pin 11 (PA13 SWDIO)                         */
/*   good GND (pin 8/18) <-> target GND (pin 8/18)                            */
/* Target just needs power (battery or USB); no DFU mode needed for SWD.      */
/*============================================================================*/

/* Header pins used as the SWD host lines (both on GPIOE). */
#define SWDH_CLK_PORT   PE2_GPIO_Port
#define SWDH_CLK_PIN    PE2_Pin        /* header pin 2  -> target SWCLK (pin 10) */
#define SWDH_IO_PORT    PE2_GPIO_Port
#define SWDH_IO_PIN     PE4_Pin        /* header pin 3  -> target SWDIO (pin 11) */

/* Last transaction diagnostics, shown on-screen. */
static uint32_t s_swd_idcode = 0;      /* DPIDR (debug-port IDCODE)            */
static uint32_t s_swd_cpuid  = 0;      /* SCB CPUID @0xE000ED00 (mem read)    */
static uint32_t s_swd_dhcsr  = 0;      /* DHCSR read-back after halt          */
static uint8_t  s_swd_ack    = 0xFF;   /* 0xFF = no transaction yet           */
static uint8_t  s_swd_pwr_ok = 0;      /* debug power-up acked                */
static uint8_t  s_swd_halt_ok = 0;     /* core reported S_HALT                 */
static int      s_swd_err = 0;         /* swdh_probe() return code (0 = OK)    */

/* SWD register addresses. */
#define SWD_DP_ABORT    0x0
#define SWD_DP_CTRL     0x4    /* CTRL/STAT (DPBANKSEL=0)                       */
#define SWD_DP_SELECT   0x8
#define SWD_DP_RDBUFF   0xC
#define SWD_AP_CSW      0x00
#define SWD_AP_TAR      0x04
#define SWD_AP_DRW      0x0C
#define SWD_ACK_OK      0x1
#define SWD_ACK_WAIT    0x2
/* On STM32H5 the Cortex-M33 core AHB-AP is AP #1 (AP #0 is the APB-AP for
 * DBGMCU etc.) — confirmed by OpenOCD's stm32h5x.cfg (-ap-num 1 for cortex_m). */
#define SWD_AHB_AP      1

/* --- STM32H5 target flash programming (over SWD/AHB-AP), from CMSIS/HAL --- */
#define TGT_FLASH_BASE      0x08000000u
#define TGT_FLASH_BANK_SZ   0x00100000u          /* 1 MB per bank             */
#define TGT_SECT_SZ         0x00002000u          /* 8 KB sector               */
#define TGT_SECT_PER_BANK   128u
#define TGT_FLASH_R         0x40022000u          /* non-secure flash reg base */
#define TGT_FL_NSKEYR       (TGT_FLASH_R + 0x04)
#define TGT_FL_NSSR         (TGT_FLASH_R + 0x20)
#define TGT_FL_NSCR         (TGT_FLASH_R + 0x28)
#define TGT_FL_NSCCR        (TGT_FLASH_R + 0x30)
#define TGT_FL_OPTSR_CUR    (TGT_FLASH_R + 0x50)
#define TGT_FL_KEY1         0x45670123u
#define TGT_FL_KEY2         0xCDEF89ABu
#define TGT_CR_LOCK         (1u << 0)
#define TGT_CR_PG           (1u << 1)
#define TGT_CR_SER          (1u << 2)
#define TGT_CR_START        (1u << 5)
#define TGT_CR_SNB_POS      6
#define TGT_CR_BKSEL        (1u << 31)
#define TGT_SR_BSY          (1u << 0)
#define TGT_SR_WBNE         (1u << 1)
#define TGT_SR_ERRMASK      ((1u<<17)|(1u<<18)|(1u<<19)|(1u<<20))  /* WRP/PGS/STRB/INC */
#define TGT_CCR_CLRALL      0x00FF0000u          /* clear EOP + all error flags */
#define RECOVERY_IMG_PATH   "0:/recovery.bin"

/* Flash-recovery progress, shown while programming. */
static uint8_t s_fl_pct   = 0;

/* Direct register line control (fast + jitter-free; bit-bang SWD is
 * host-clocked so exact timing is not critical, only edge order). */
static inline void swdh_clk(int v)
{
    SWDH_CLK_PORT->BSRR = v ? (uint32_t)SWDH_CLK_PIN
                            : ((uint32_t)SWDH_CLK_PIN << 16);
}
static inline void swdh_io(int v)
{
    SWDH_IO_PORT->BSRR = v ? (uint32_t)SWDH_IO_PIN
                           : ((uint32_t)SWDH_IO_PIN << 16);
}
static inline int swdh_get(void)
{
    return (SWDH_IO_PORT->IDR & SWDH_IO_PIN) ? 1 : 0;
}

/* Half-bit delay count. Reset to the reliable value at every connect; the
 * bulk erase/program lowers it once the link is proven (see swdh_flash_recover).*/
static int s_swd_dly = 40;

static inline void swdh_delay(void)
{
    volatile int i;
    for (i = 0; i < s_swd_dly; i++) __NOP();
}

/* Point the shared SWDIO line as host output (out=1) or target input (out=0). */
static void swdh_io_dir(int out)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = SWDH_IO_PIN;
    gi.Mode  = out ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    gi.Pull  = out ? GPIO_NOPULL         : GPIO_PULLUP;   /* idle-high while target drives */
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWDH_IO_PORT, &gi);
}

/* Write one bit (host driving SWDIO): target samples on the rising edge. */
static void swdh_wr_bit(int b)
{
    swdh_io(b);
    swdh_clk(0); swdh_delay();
    swdh_clk(1); swdh_delay();
}
/* Read one bit (target driving SWDIO): sample while clock is low, then rise. */
static int swdh_rd_bit(void)
{
    int b;
    swdh_clk(0); swdh_delay();
    b = swdh_get();
    swdh_clk(1); swdh_delay();
    return b;
}
/* One clock with SWDIO left alone — used for the turnaround cycles. */
static void swdh_trn(void)
{
    swdh_clk(0); swdh_delay();
    swdh_clk(1); swdh_delay();
}
static void swdh_wr(uint32_t val, int n)
{
    while (n--) { swdh_wr_bit((int)(val & 1u)); val >>= 1; }
}
static uint32_t swdh_rd(int n)
{
    uint32_t v = 0; int i;
    for (i = 0; i < n; i++) if (swdh_rd_bit()) v |= (1u << i);
    return v;
}

/* >=50 clocks with SWDIO high resets the SWD line state machine. */
static void swdh_line_reset(void)
{
    int i;
    swdh_io_dir(1);
    swdh_io(1);
    for (i = 0; i < 56; i++) swdh_wr_bit(1);
}

/* Bring up the target debug port: line reset -> JTAG-to-SWD select (0xE79E)
 * -> line reset -> a few idle bits, per ADIv5 SW-DP. */
static void swdh_connect(void)
{
    GPIO_InitTypeDef gi = {0};

    s_swd_dly = 40;            /* always handshake at the reliable slow speed */
    gi.Pin   = SWDH_CLK_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWDH_CLK_PORT, &gi);
    swdh_clk(1);

    swdh_io_dir(1);
    swdh_line_reset();
    swdh_wr(0xE79E, 16);        /* JTAG-to-SWD switch sequence, LSB first */
    swdh_line_reset();
    swdh_io(0);
    swdh_wr(0, 8);              /* idle cycles */
}

/* Generic SWD transfer. apndp: 0=DP,1=AP. rnw: 0=write,1=read. a=register
 * address (bits[3:2] select the register). For reads, *data receives the
 * result; for writes, *data supplies it. Returns the 3-bit ACK (OK=0b001). */
static uint8_t swdh_xfer(int apndp, int rnw, uint8_t a, uint32_t *data)
{
    int a2 = (a >> 2) & 1, a3 = (a >> 3) & 1;
    int par = (apndp ^ rnw ^ a2 ^ a3) & 1;
    uint32_t req = 1u | ((uint32_t)apndp << 1) | ((uint32_t)rnw << 2)
                 | ((uint32_t)a2 << 3) | ((uint32_t)a3 << 4)
                 | ((uint32_t)par << 5) | (1u << 7);   /* Start .. Park */
    uint8_t ack;
    uint32_t d, calc = 0;
    int i, rp;

    swdh_io_dir(1);
    swdh_wr(req, 8);

    swdh_io_dir(0);
    swdh_trn();                          /* turnaround host -> target */
    ack = (uint8_t)swdh_rd(3);           /* ACK, LSB first */

    if (ack != SWD_ACK_OK)
    {
        swdh_trn();                      /* return the bus, bail */
        swdh_io_dir(1);
        return ack;
    }

    if (rnw)                             /* READ: target keeps driving data */
    {
        d = swdh_rd(32);
        rp = swdh_rd_bit();
        swdh_trn();                      /* turnaround target -> host */
        swdh_io_dir(1);
        if (data) *data = d;
        for (i = 0; i < 32; i++) calc ^= (d >> i) & 1u;
        if ((calc & 1u) != (uint32_t)rp) return 0x8;   /* flag parity error */
    }
    else                                 /* WRITE: hand bus back, drive data */
    {
        swdh_trn();                      /* turnaround target -> host */
        swdh_io_dir(1);
        d = data ? *data : 0;
        swdh_wr(d, 32);
        for (i = 0; i < 32; i++) calc ^= (d >> i) & 1u;
        swdh_wr_bit((int)(calc & 1u));
    }
    return ack;                          /* SWD_ACK_OK on success */
}

/* Retrying transfer: a fresh jumper link and the DAP right after power-up can
 * answer WAIT (or miss a bit); re-issue until OK or the budget runs out. */
static uint8_t swdh_try(int apndp, int rnw, uint8_t a, uint32_t *data)
{
    uint8_t ack = 0; int k;
    for (k = 0; k < 8; k++)
    {
        ack = swdh_xfer(apndp, rnw, a, data);
        if (ack == SWD_ACK_OK) break;
    }
    return ack;
}

static uint8_t swdh_dp_write(uint8_t a, uint32_t d) { return swdh_try(0, 0, a, &d); }
static uint8_t swdh_dp_read (uint8_t a, uint32_t *d){ return swdh_try(0, 1, a, d);  }

/* Point DP.SELECT at the core AHB-AP (APSEL = SWD_AHB_AP) with the AP-register
 * bank in APBANKSEL. CSW/TAR/DRW are all in bank 0, so APBANKSEL = reg high nibble. */
static void swdh_ap_select(uint8_t apreg)
{
    uint32_t sel = ((uint32_t)SWD_AHB_AP << 24) | (uint32_t)(apreg & 0xF0);
    swdh_dp_write(SWD_DP_SELECT, sel);
}
static uint8_t swdh_ap_write(uint8_t apreg, uint32_t d)
{
    swdh_ap_select(apreg);
    return swdh_try(1, 0, apreg, &d);
}
/* AP reads are pipelined one behind: issue the AP read, then take the real
 * value from RDBUFF. */
static uint8_t swdh_ap_read(uint8_t apreg, uint32_t *d)
{
    uint32_t dummy;
    swdh_ap_select(apreg);
    swdh_try(1, 1, apreg, &dummy);
    return swdh_dp_read(SWD_DP_RDBUFF, d);
}

static uint8_t swdh_mem_read(uint32_t addr, uint32_t *val)
{
    swdh_ap_write(SWD_AP_TAR, addr);
    return swdh_ap_read(SWD_AP_DRW, val);
}
static uint8_t swdh_mem_write(uint32_t addr, uint32_t val)
{
    swdh_ap_write(SWD_AP_TAR, addr);
    return swdh_ap_write(SWD_AP_DRW, val);
}

/* Full Phase-2 bring-up: connect, read DPIDR, power the debug domain, read
 * CPUID over the AHB-AP (proves memory READ), halt the core and read DHCSR
 * back (proves memory WRITE), then release the core so a live target keeps
 * running. Returns 0 on a fully successful link. */
static int swdh_probe(void)
{
    uint32_t v;
    int tries;

    s_swd_idcode = s_swd_cpuid = s_swd_dhcsr = 0;
    s_swd_ack = 0xFF; s_swd_pwr_ok = 0; s_swd_halt_ok = 0;

    /* DP IDCODE is required as the first DP read after a line reset. A marginal
     * jumper link can miss the very first attempt, so re-run the full connect a
     * few times until we get a valid, non-empty IDCODE. */
    for (tries = 0; tries < 12; tries++)
    {
        swdh_connect();
        s_swd_ack = swdh_dp_read(0x0, &s_swd_idcode);
        if (s_swd_ack == SWD_ACK_OK &&
            s_swd_idcode != 0 && s_swd_idcode != 0xFFFFFFFF)
            break;
    }
    if (s_swd_ack != SWD_ACK_OK ||
        s_swd_idcode == 0 || s_swd_idcode == 0xFFFFFFFF)
        return -1;

    /* Clear any sticky error/overrun flags, then power up debug. */
    swdh_dp_write(SWD_DP_ABORT, 0x1E);
    swdh_dp_write(SWD_DP_SELECT, 0x0);          /* DPBANKSEL=0, APSEL=0 */
    swdh_dp_write(SWD_DP_CTRL, 0x50000000u);    /* CDBGPWRUPREQ|CSYSPWRUPREQ */
    for (tries = 0; tries < 50; tries++)
    {
        if (swdh_dp_read(SWD_DP_CTRL, &v) == SWD_ACK_OK &&
            (v & 0xA0000000u) == 0xA0000000u)   /* CDBGPWRUPACK|CSYSPWRUPACK */
        {
            s_swd_pwr_ok = 1;
            break;
        }
    }
    if (!s_swd_pwr_ok) return -2;

    /* Configure the core AHB-AP for 32-bit, auto-incrementing accesses with
     * STM32H5 non-secure privileged Prot bits (SPROT=1). Value 0x4B000000 in
     * the Prot field matches OpenOCD's non-secure apcsw for this part; low bits
     * set Size=32-bit + AddrInc=single. */
    if (swdh_ap_read(SWD_AP_CSW, &v) != SWD_ACK_OK) return -3;
    v = (v & ~0x4F000037u) | 0x4B000012u;
    swdh_ap_write(SWD_AP_CSW, v);

    /* Memory READ: SCB CPUID (top bytes 0x410FD2xx = Cortex-M33). */
    if (swdh_mem_read(0xE000ED00u, &s_swd_cpuid) != SWD_ACK_OK) return -4;

    /* Memory WRITE: halt the core via DHCSR, then confirm S_HALT. */
    swdh_mem_write(0xE000EDF0u, 0xA05F0003u);   /* DBGKEY|C_DEBUGEN|C_HALT */
    for (tries = 0; tries < 20; tries++)
    {
        if (swdh_mem_read(0xE000EDF0u, &s_swd_dhcsr) == SWD_ACK_OK &&
            (s_swd_dhcsr & (1u << 17)))          /* S_HALT */
        {
            s_swd_halt_ok = 1;
            break;
        }
    }

    /* Release the core (clear halt + debug enable) so a live test target keeps
     * running. A truly bricked target won't run anyway; this is just courtesy. */
    swdh_mem_write(0xE000EDF0u, 0xA05F0000u);

    return s_swd_halt_ok ? 0 : -5;
}

/*----------------------------------------------------------------------------*/
/*  Phase 3: erase + program the target's flash from an SD image over SWD.     */
/*----------------------------------------------------------------------------*/

/* Fast AP helpers for the flash loop: SELECT is left pointing at the AHB-AP
 * (bank 0) the whole time, so TAR/DRW/CSW accesses skip the per-op re-select. */
static void swdh_ap_lock(void)
{
    swdh_dp_write(SWD_DP_SELECT, ((uint32_t)SWD_AHB_AP << 24));
}
static uint8_t swdh_apw(uint8_t reg, uint32_t d) { return swdh_try(1, 0, reg, &d); }
static uint8_t swdh_apr(uint8_t reg, uint32_t *d)
{
    uint32_t dummy;
    swdh_try(1, 1, reg, &dummy);
    return swdh_dp_read(SWD_DP_RDBUFF, d);
}
/* Single-word target memory access (SELECT already locked to the AHB-AP). */
static uint8_t swdh_mw(uint32_t a, uint32_t v) { swdh_apw(SWD_AP_TAR, a); return swdh_apw(SWD_AP_DRW, v); }
static uint8_t swdh_mr(uint32_t a, uint32_t *v){ swdh_apw(SWD_AP_TAR, a); return swdh_apr(SWD_AP_DRW, v); }

/* Wait for the target flash to finish an operation (BSY + write-buffer clear). */
static int tgt_flash_wait(void)
{
    uint32_t sr; int i;
    for (i = 0; i < 200000; i++)
    {
        if (swdh_mr(TGT_FL_NSSR, &sr) == SWD_ACK_OK &&
            !(sr & (TGT_SR_BSY | TGT_SR_WBNE)))
            return (sr & TGT_SR_ERRMASK) ? -1 : 0;
    }
    return -2;   /* timeout */
}

/* Draw a one-line progress screen for the (blocking) flash operation. */
static void tgt_flash_draw(const char *stage)
{
    char line[28];
    m1_u8g2_firstpage();
    do {
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 9, "Flashing peer...");
        u8g2_DrawHLine(&m1_u8g2, 0, 12, 128);
        u8g2_DrawStr(&m1_u8g2, 2, 30, stage);
        snprintf(line, sizeof(line), "%u%%", (unsigned)s_fl_pct);
        u8g2_DrawStr(&m1_u8g2, 2, 44, line);
        u8g2_DrawStr(&m1_u8g2, 2, 61, "Do NOT disconnect");
    } while (m1_u8g2_nextpage());
}

/* Erase both target banks, then program RECOVERY_IMG_PATH at 0x08000000 and
 * reset the target. Returns 0 on success, else a negative stage code. Requires
 * a halted core with the AHB-AP CSW already configured (call after swdh_probe).*/
static int swdh_flash_recover(const char *path)
{
    static uint32_t buf[256];     /* 1 KB SD read chunk (4-byte aligned) — matches
                                   * the codebase's proven FW_IMAGE_CHUNK_SIZE; larger
                                   * reads crashed on this SDMMC/FatFS setup.       */
    uint8_t *b = (uint8_t *)buf;
    FIL fp; UINT br; FRESULT fr;
    uint32_t cr, fsize, addr, done, bank;
    int s;
    uint8_t last_pct = 0xFF;      /* only redraw the screen when % actually changes */

    /* Re-establish the link and leave the core HALTED (probe releases it). */
    s_fl_pct = 0;
    if (swdh_probe() != 0) return -10;
    swdh_ap_lock();
    swdh_mw(0xE000EDF0u, 0xA05F0003u);       /* halt: DBGKEY|C_DEBUGEN|C_HALT   */

    /* Link is proven — run the bulk erase/program at a faster clock. The next
     * connect resets s_swd_dly to the reliable value. Safety: transfer retry +
     * read-parity checks here, and the target's own boot-CRC after. */
    s_swd_dly = 15;

    /* Unlock the flash control register. */
    tgt_flash_draw("Unlocking...");
    swdh_mw(TGT_FL_NSKEYR, TGT_FL_KEY1);
    swdh_mw(TGT_FL_NSKEYR, TGT_FL_KEY2);
    if (swdh_mr(TGT_FL_NSCR, &cr) != SWD_ACK_OK || (cr & TGT_CR_LOCK)) return -11;

    /* Erase both banks sector-by-sector (clean slate, swap-agnostic). */
    for (s = 0; s < (int)(2 * TGT_SECT_PER_BANK); s++)
    {
        uint32_t sect = (uint32_t)(s % TGT_SECT_PER_BANK);
        bank = (s < (int)TGT_SECT_PER_BANK) ? 0u : TGT_CR_BKSEL;
        swdh_mw(TGT_FL_NSCR, TGT_CR_SER | (sect << TGT_CR_SNB_POS) | bank | TGT_CR_START);
        if (tgt_flash_wait() != 0) return -12;
        swdh_mw(TGT_FL_NSCCR, TGT_CCR_CLRALL);
        m1_wdt_reset();          /* keep the IWDG alive through the tight loop */
        s_fl_pct = (uint8_t)((s + 1) * 100 / (2 * TGT_SECT_PER_BANK));
        if ((s & 7) == 0) tgt_flash_draw("Erasing...");
    }
    swdh_mw(TGT_FL_NSCR, 0);                  /* clear SER                       */

    /* Program the image (128-bit quad-words; skip already-erased 0xFF groups). */
    fr = f_open(&fp, path, FA_READ);
    if (fr != FR_OK) return -20;
    fsize = (uint32_t)f_size(&fp);
    if (fsize == 0 || fsize > TGT_FLASH_BANK_SZ) { f_close(&fp); return -21; }

    s_fl_pct = 0;
    swdh_mw(TGT_FL_NSCR, TGT_CR_PG);
    addr = TGT_FLASH_BASE;
    done = 0;
    while (done < fsize)
    {
        UINT want = (fsize - done > sizeof(buf)) ? (UINT)sizeof(buf) : (UINT)(fsize - done);
        UINT off;
        if (f_read(&fp, b, want, &br) != FR_OK || br != want) { f_close(&fp); return -22; }
        while (br & 15u) b[br++] = 0xFF;      /* pad tail up to a quad-word      */

        for (off = 0; off < br; off += 16)
        {
            uint32_t *w = (uint32_t *)(b + off);
            if ((off & 0x1FFu) == 0) m1_wdt_reset();   /* kick IWDG every 512 B */
            if (!(w[0]==0xFFFFFFFFu && w[1]==0xFFFFFFFFu &&
                  w[2]==0xFFFFFFFFu && w[3]==0xFFFFFFFFu))
            {
                swdh_apw(SWD_AP_TAR, addr);
                swdh_apw(SWD_AP_DRW, w[0]);
                swdh_apw(SWD_AP_DRW, w[1]);
                swdh_apw(SWD_AP_DRW, w[2]);
                swdh_apw(SWD_AP_DRW, w[3]);
                if (tgt_flash_wait() != 0) { f_close(&fp); return -23; }
                /* No per-word flag clear: tgt_flash_wait() already aborts on any
                 * error bit, and a leftover EOP doesn't block the next program. */
            }
            addr += 16;
        }
        done += want;
        s_fl_pct = (uint8_t)(done * 100 / fsize);
        if (s_fl_pct != last_pct) { last_pct = s_fl_pct; tgt_flash_draw("Programming..."); }
    }
    swdh_mw(TGT_FL_NSCR, TGT_CR_LOCK);        /* clear PG + relock the flash     */

    /* Sanity-verify the vector table: SP in RAM, reset vector in flash. */
    tgt_flash_draw("Verifying...");
    {
        uint32_t sp = 0, rv = 0;
        f_lseek(&fp, 0);
        f_read(&fp, b, 8, &br);
        (void)swdh_mr(TGT_FLASH_BASE + 0, &sp);
        (void)swdh_mr(TGT_FLASH_BASE + 4, &rv);
        f_close(&fp);
        if (sp != buf[0] || rv != buf[1]) return -30;   /* readback mismatch     */
        if ((sp & 0xFF000000u) != 0x20000000u) return -31;  /* SP not in SRAM    */
    }

    /* Release debug and reset the target so it boots the fresh firmware. */
    s_fl_pct = 100;
    swdh_mw(0xE000EDF0u, 0xA05F0000u);        /* clear halt + debug enable       */
    swdh_mw(0xE000ED0Cu, 0x05FA0004u);        /* AIRCR SYSRESETREQ               */
    return 0;
}

#ifndef M1_RECOVERY_BUILD
/* Browse the SD card and pick a firmware .bin to flash to the peer. Fills 'out'
 * with the full path and returns 1; returns 0 if the user backs out. Only in the
 * working FW (the recovery FW strips the file browser and never runs this tool). */
static int tool_pick_flash_file(char *out, size_t cap)
{
    S_M1_Main_Q_t q_item;
    S_M1_Buttons_Status btn;
    S_M1_file_info *fi;

    m1_fb_init(&m1_u8g2);
    m1_fb_set_dir("0:/");
    m1_fb_display(NULL);

    for (;;)
    {
        if (xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE) continue;
        if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;
        if (xQueueReceive(button_events_q_hdl, &btn, 0) != pdTRUE) continue;

        if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            m1_fb_deinit();
            return 0;                    /* cancelled */
        }

        fi = m1_fb_display(&btn);
        if (fi->status == FB_OK && fi->file_is_selected)
        {
            size_t n = strlen(fi->file_name);
            if (n < 4 || strncmp(&fi->file_name[n - 4], ".bin", 4) != 0)
                continue;                /* not a .bin — keep browsing */
            snprintf(out, cap, "%s/%s", fi->dir_name, fi->file_name);
            m1_fb_deinit();
            return 1;
        }
    }
}
#endif /* !M1_RECOVERY_BUILD */

void tool_peer_recover(void)
{
    S_M1_Main_Q_t q_item;
    S_M1_Buttons_Status btn;
    BaseType_t ret;
    int state = 0;   /* 0 instr, 1 linked, 2 confirm, 3 flash-result, -1 fail */
    int fl_err = 0;           /* swdh_flash_recover() result                     */
    char flash_path[64] = RECOVERY_IMG_PATH;   /* file to flash (picked in working FW) */
    char line[28];
    bool redraw = true, do_probe = false;

    while (1)
    {
        if (do_probe)
        {
            do_probe = false;
            s_swd_err = swdh_probe();
            state = (s_swd_err == 0) ? 1 : -1;
            redraw = true;
        }

        if (redraw)
        {
            redraw = false;
            m1_u8g2_firstpage();
            do {
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
                u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
                u8g2_DrawStr(&m1_u8g2, 2, 9, "Recover Peer (SWD)");
                u8g2_DrawHLine(&m1_u8g2, 0, 12, 128);

                if (state == 0)
                {
                    u8g2_DrawStr(&m1_u8g2, 2, 23, "P2>tgt10 P3>tgt11");
                    u8g2_DrawStr(&m1_u8g2, 2, 34, "GND<->GND, tgt powered");
                    u8g2_DrawStr(&m1_u8g2, 2, 45, "(no DFU needed)");
                    u8g2_DrawStr(&m1_u8g2, 2, 61, "OK:Connect  Back:Exit");
                }
                else if (state == 1)
                {
                    snprintf(line, sizeof(line), "ID :0x%08lX", (unsigned long)s_swd_idcode);
                    u8g2_DrawStr(&m1_u8g2, 2, 23, line);
                    snprintf(line, sizeof(line), "CPU:0x%08lX", (unsigned long)s_swd_cpuid);
                    u8g2_DrawStr(&m1_u8g2, 2, 34, line);
                    u8g2_DrawStr(&m1_u8g2, 2, 45, "Linked & halted OK");
                    u8g2_DrawStr(&m1_u8g2, 2, 61, "OK:Pick FW  Back:Exit");
                }
                else if (state == 2)
                {
                    const char *fn = strrchr(flash_path, '/');
                    fn = fn ? fn + 1 : flash_path;
                    u8g2_DrawStr(&m1_u8g2, 2, 24, "FLASH this file?");
                    u8g2_DrawStr(&m1_u8g2, 2, 35, "ERASES target flash!");
                    snprintf(line, sizeof(line), "%s", fn);
                    u8g2_DrawStr(&m1_u8g2, 2, 46, line);
                    u8g2_DrawStr(&m1_u8g2, 2, 61, "OK:GO  Back:Cancel");
                }
                else if (state == 3)
                {
                    if (fl_err == 0) {
                        u8g2_DrawStr(&m1_u8g2, 2, 26, "Flash OK! Target reset.");
                        u8g2_DrawStr(&m1_u8g2, 2, 40, "Recovery complete.");
                    } else {
                        snprintf(line, sizeof(line), "Flash FAIL @ %d", fl_err);
                        u8g2_DrawStr(&m1_u8g2, 2, 26, line);
                        u8g2_DrawStr(&m1_u8g2, 2, 40,
                            (fl_err <= -20 && fl_err > -30) ? "SD/file problem" :
                            (fl_err <= -30) ? "verify mismatch" :
                            (fl_err == -11) ? "flash unlock failed" :
                            (fl_err == -12) ? "erase failed" :
                            (fl_err == -23) ? "program failed" : "link lost");
                    }
                    u8g2_DrawStr(&m1_u8g2, 2, 61, "OK:Menu  Back:Exit");
                }
                else
                {
                    snprintf(line, sizeof(line), "Fail@%d ack:0x%X pwr:%s",
                             s_swd_err, (unsigned)s_swd_ack, s_swd_pwr_ok ? "OK" : "no");
                    u8g2_DrawStr(&m1_u8g2, 2, 23, line);
                    snprintf(line, sizeof(line), "ID :0x%08lX", (unsigned long)s_swd_idcode);
                    u8g2_DrawStr(&m1_u8g2, 2, 34, line);
                    snprintf(line, sizeof(line), "CPU:0x%08lX", (unsigned long)s_swd_cpuid);
                    u8g2_DrawStr(&m1_u8g2, 2, 45, line);
                    u8g2_DrawStr(&m1_u8g2, 2, 61, "OK:Retry  Back:Exit");
                }
            } while (m1_u8g2_nextpage());
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &btn, 0);
            if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (state == 2)      { state = 1; redraw = true; }   /* cancel flash */
                else                 { xQueueReset(main_q_hdl); break; }
            }
            else if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (state == 1)
                {
#ifndef M1_RECOVERY_BUILD
                    /* Pick the firmware .bin to flash (blocks in the file browser).
                     * Cancel returns to the linked screen. */
                    if (tool_pick_flash_file(flash_path, sizeof(flash_path)))
                        state = 2;
                    redraw = true;
#else
                    state = 2; redraw = true;
#endif
                }
                else if (state == 2)                                 /* flash confirmed  */
                {
                    fl_err = swdh_flash_recover(flash_path);
                    state = 3;
                    redraw = true;
                }
                else if (state == 3) { state = 0; redraw = true; }   /* back to menu    */
                else                 { do_probe = true; }            /* 0 / -1: probe   */
            }
        }
    }

    /* Park both host lines back to inputs so we stop driving the target's
     * debug pins once we leave the tool. */
    swdh_io_dir(0);
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin  = SWDH_CLK_PIN;
        gi.Mode = GPIO_MODE_INPUT;
        gi.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(SWDH_CLK_PORT, &gi);
    }
}



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
