/* See COPYING.txt for license details. */

/*
 * m1_field_detect.c
 *
 * Passive field detection for NFC (13.56 MHz) and LF RFID (125 kHz).
 *
 * NFC:  Uses the ST25R3916's built-in External Field Detector (EFD).
 *       rfalIsExtFieldOn() reads AUX_DISPLAY register bit 6 (efd_o).
 *
 * RFID: Uses ADC1 channel 15 (PA3 / RFID_RF_IN) to measure the analog
 *       voltage on the RFID antenna. The M1's RFID circuit includes an
 *       envelope detector, so PA3 carries a DC level that shifts when
 *       an external reader field is present. We compare the current
 *       average ADC reading against a baseline taken at startup.
 */

/*************************** I N C L U D E S **********************************/
#include <stdbool.h>
#include <string.h>

#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_field_detect.h"

/* NFC / RFAL */
#include "rfal_nfc.h"
#include "rfal_rf.h"
#include "st25r3916.h"
#include "st25r3916_com.h"
#include "nfc_conf.h"

/*************************** D E F I N E S ************************************/

/* ADC parameters for RFID detection */
#define ADC_SAMPLES          512     /* samples per measurement (averaged) */
#define ADC_BASELINE_SAMPLES 1024    /* samples for initial baseline */
#define ADC_DC_THRESHOLD     15      /* min DC shift from baseline to detect field */
                                     /* 12-bit ADC, 3.3V range: ~12mV per count */
                                     /* 15 counts = ~180mV shift threshold */

/*************************** V A R I A B L E S ********************************/

static bool field_detect_active = false;
static bool nfc_chip_ok = false;
static bool adc_initialized = false;
static int  adc_baseline = -1;        /* baseline ADC average (no field) */
static int  last_rfid_avg = 0;        /* last ADC average reading */
static int  last_rfid_delta = 0;      /* last |avg - baseline| (debug) */
static int  last_nfc_aux = 0;         /* last AUX_DISPLAY register value */
static int  last_nfc_opctl = 0;      /* OP_CONTROL readback (debug) */

/* Declared in app_x-cube-nfcx.c */
extern EXTI_HandleTypeDef USR_INT_LINE;
extern void st25r3916Isr(void);

/*************************** A D C   ( bare metal ) ***************************/

static void adc_init(void)
{
    /* Enable ADC1 clock */
    __HAL_RCC_ADC_CLK_ENABLE();

    /* Configure PA3 as analog mode (MODER = 11) */
    GPIOA->MODER |= (3UL << (3 * 2));
    GPIOA->PUPDR &= ~(3UL << (3 * 2));

    /* Exit deep power-down mode */
    ADC1->CR &= ~ADC_CR_DEEPPWD;

    /* Enable internal voltage regulator */
    ADC1->CR |= ADC_CR_ADVREGEN;

    /* Wait for regulator startup (~10 us) */
    for (volatile int i = 0; i < 25000; i++) { __asm volatile("nop"); }

    /* Configure: single conversion, right-aligned, 12-bit */
    ADC1->CFGR = 0;
    ADC1->CFGR2 = 0;

    /* Sampling time for channel 15: 010 = 7.5 ADC clocks */
    ADC1->SMPR2 = (2UL << ADC_SMPR2_SMP15_Pos);

    /* Sequence: 1 conversion, channel 15 in rank 1 */
    ADC1->SQR1 = (15UL << ADC_SQR1_SQ1_Pos);

    /* ADC common: CKMODE = 01 (synchronous HCLK/1) */
    ADC12_COMMON->CCR = (1UL << 18);

    /* Clear ADRDY flag */
    ADC1->ISR = ADC_ISR_ADRDY;

    /* Enable ADC */
    ADC1->CR |= ADC_CR_ADEN;

    /* Wait for ADRDY */
    uint32_t timeout = HAL_GetTick() + 100;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    {
        if (HAL_GetTick() > timeout)
            break;
    }

    adc_initialized = true;
}

static void adc_deinit(void)
{
    if (!adc_initialized)
        return;

    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN) {}

    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_DEEPPWD;

    __HAL_RCC_ADC_CLK_DISABLE();

    /* Restore PA3 to input mode */
    GPIOA->MODER &= ~(3UL << (3 * 2));

    adc_initialized = false;
}

/* Read a single ADC sample (blocking) */
static uint16_t adc_read_single(void)
{
    ADC1->ISR = ADC_ISR_EOC;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    return (uint16_t)(ADC1->DR & 0xFFF);
}

/* Read N samples and return average */
static int adc_read_average(int num_samples)
{
    uint32_t sum = 0;
    for (int i = 0; i < num_samples; i++)
        sum += adc_read_single();
    return (int)(sum / (uint32_t)num_samples);
}


/*************************** F U N C T I O N S ********************************/

/*============================================================================*/
int m1_field_detect_start(void)
{
    if (field_detect_active)
        return nfc_chip_ok ? 0 : -1;

    nfc_chip_ok = false;

    /* Power up external rails */
    HAL_GPIO_WritePin(EN_EXT_5V_GPIO_Port, EN_EXT_5V_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_EXT_3V3_GPIO_Port, EN_EXT_3V3_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    /* --- NFC: Initialize ST25R3916 --- */
    USR_INT_LINE.Line = USR_INT_LINE_NUM;
    USR_INT_LINE.RisingCallback = st25r3916Isr;
    HAL_EXTI_GetHandle(&USR_INT_LINE, USR_INT_LINE.Line);
    HAL_EXTI_RegisterCallback(&USR_INT_LINE, HAL_EXTI_COMMON_CB_ID,
                              USR_INT_LINE.RisingCallback);

    for (int i = 0; i < 5; i++)
    {
        if (rfalNfcInitialize() == RFAL_ERR_NONE)
        {
            nfc_chip_ok = true;
            break;
        }
        HAL_Delay(20);
    }

    if (nfc_chip_ok)
    {
        /*
         * rfalNfcInitialize() sets EFD to "auto" mode, but auto only
         * samples during chip state machine transitions (e.g. collision
         * avoidance).  Since we never start a discovery loop, efd_o
         * stays stale at 0.
         *
         * Switch to manual EFD (Peak Detector Threshold mode) so that
         * efd_o in AUX_DISPLAY continuously reflects the comparator
         * output — exactly what we need for passive field detection.
         */
        st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL,
                                    ST25R3916_REG_OP_CONTROL_en_fd_mask,
                                    ST25R3916_REG_OP_CONTROL_en_fd_manual_efd_pdt);

        /* Set lowest EFD activation threshold for maximum sensitivity:
         * trg = 75mV (0x0), rfe = 25mV (0x8) */
        st25r3916WriteRegister(ST25R3916_REG_FIELD_THRESHOLD_ACTV,
                               ST25R3916_REG_FIELD_THRESHOLD_ACTV_trg_75mV |
                               ST25R3916_REG_FIELD_THRESHOLD_ACTV_rfe_25mV);
        st25r3916WriteRegister(ST25R3916_REG_FIELD_THRESHOLD_DEACTV,
                               ST25R3916_REG_FIELD_THRESHOLD_DEACTV_trg_75mV |
                               ST25R3916_REG_FIELD_THRESHOLD_DEACTV_rfe_25mV);
    }

    /* --- RFID: Initialize ADC on PA3 --- */
    HAL_GPIO_WritePin(RFID_PULL_GPIO_Port, RFID_PULL_Pin, GPIO_PIN_RESET);
    adc_init();

    /* Take baseline reading (no reader should be near at startup) */
    if (adc_initialized)
    {
        /* Let the circuit settle after ADC init */
        HAL_Delay(20);
        adc_baseline = adc_read_average(ADC_BASELINE_SAMPLES);
    }

    field_detect_active = true;
    return nfc_chip_ok ? 0 : -1;
}

/*============================================================================*/
void m1_field_detect_stop(void)
{
    if (!field_detect_active)
        return;

    adc_deinit();

    if (nfc_chip_ok)
    {
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        st25r3916Deinitialize();
    }

    HAL_GPIO_WritePin(EN_EXT_5V_GPIO_Port, EN_EXT_5V_Pin, GPIO_PIN_RESET);

    field_detect_active = false;
    nfc_chip_ok = false;
    adc_baseline = -1;
}

/*============================================================================*/
bool m1_field_detect_nfc(void)
{
    if (!field_detect_active || !nfc_chip_ok)
    {
        last_nfc_aux = -1;
        last_nfc_opctl = -1;
        return false;
    }

    /* Re-apply EFD config each poll — the RFAL ISR can reset OP_CONTROL */
    st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL,
                                ST25R3916_REG_OP_CONTROL_en_fd_mask,
                                ST25R3916_REG_OP_CONTROL_en_fd_manual_efd_pdt);

    /* Read back OP_CONTROL for diagnostics */
    {
        uint8_t opctl = 0;
        st25r3916ReadRegister(ST25R3916_REG_OP_CONTROL, &opctl);
        last_nfc_opctl = (int)opctl;
    }

    /*
     * Multi-sample: NFC readers (like Flipper Zero) poll in short bursts —
     * field ON for ~5ms, OFF for ~30ms.  A single register read would miss
     * the field most of the time.  Sample 100 times over ~50ms to span
     * at least one full polling cycle.  Return true if ANY sample sees efd_o.
     */
    bool detected = false;
    uint8_t last_reg = 0;

    for (int i = 0; i < 100; i++)
    {
        uint8_t reg = 0;
        st25r3916ReadRegister(ST25R3916_REG_AUX_DISPLAY, &reg);
        last_reg = reg;

        if (reg & ST25R3916_REG_AUX_DISPLAY_efd_o)
        {
            detected = true;
            last_nfc_aux = (int)reg;
            break;  /* found it — no need to keep sampling */
        }

        /* ~500 µs delay between samples → 100 samples ≈ 50 ms window */
        for (volatile int j = 0; j < 5000; j++) { __asm volatile("nop"); }
    }

    if (!detected)
        last_nfc_aux = (int)last_reg;

    return detected;
}

/*============================================================================*/
bool m1_field_detect_rfid(uint32_t *frequency)
{
    if (!field_detect_active || !adc_initialized || adc_baseline < 0)
    {
        last_rfid_delta = -1;
        return false;
    }

    /*
     * Measure the current average ADC level on PA3.
     * The RFID circuit's envelope detector produces a DC level that
     * shifts when an external reader field is present:
     *   - No field:    PA3 reads near baseline
     *   - Field present: PA3 shifts up or down from baseline
     *
     * We check the absolute difference from baseline.
     */
    last_rfid_avg = adc_read_average(ADC_SAMPLES);

    int delta = last_rfid_avg - adc_baseline;
    if (delta < 0) delta = -delta;
    last_rfid_delta = delta;

    if (delta >= ADC_DC_THRESHOLD)
    {
        if (frequency)
            *frequency = 125000;
        return true;
    }

    if (frequency)
        *frequency = 0;
    return false;
}

/*============================================================================*/
int m1_field_detect_rfid_raw(void)
{
    return last_rfid_delta;
}

/*============================================================================*/
int m1_field_detect_nfc_raw(void)
{
    /* Returns the cached AUX_DISPLAY value from the last m1_field_detect_nfc()
     * call.  Must call nfc() first in each poll cycle. */
    return last_nfc_aux;
}

/*============================================================================*/
int m1_field_detect_nfc_opctl(void)
{
    return last_nfc_opctl;
}
