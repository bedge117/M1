/* See COPYING.txt for license details. */
/*
 * m1_rng.c — STM32H573 hardware TRNG via direct register access. See m1_rng.h.
 *
 * The RNG kernel clock is HSI48 (already enabled at boot for USB — SystemClock
 * config sets RCC_HSI48_ON), which is the RNG's reset-default source, so bring-up
 * is just: select HSI48, enable the bus clock, set RNGEN keeping the reset CR
 * (which already holds the NIST-recommended health config), and wait for DRDY.
 * Any seed/clock-error flag or timeout demotes to a rand() fallback permanently,
 * so this function can never block the caller.
 */
#include "m1_rng.h"
#include "stm32h5xx_hal.h"     /* CMSIS device (RNG/RCC) + HAL_GetTick */
#include <stdlib.h>

#define RNG_SPIN_MAX   200000u

static int s_state;            /* 0 = uninit, 1 = TRNG ok, 2 = rand() fallback */

static void rng_hw_init(void)
{
    RCC->CCIPR5  &= ~RCC_CCIPR5_RNGSEL;   /* RNG kernel clock = HSI48 (reset default) */
    RCC->AHB2ENR |=  RCC_AHB2ENR_RNGEN;   /* enable RNG peripheral bus clock */
    (void)RCC->AHB2ENR;                   /* read-back barrier: clock is up before use */

    RNG->CR |= RNG_CR_RNGEN;              /* keep reset (recommended) config, enable core */

    for (volatile uint32_t i = 0; i < RNG_SPIN_MAX; i++) {
        uint32_t sr = RNG->SR;
        if (sr & (RNG_SR_SEIS | RNG_SR_CEIS)) { s_state = 2; return; } /* seed/clock error */
        if (sr & RNG_SR_DRDY)                 { s_state = 1; return; } /* first word ready */
    }
    s_state = 2;                          /* timed out -> fallback */
}

static uint32_t sw_rand(void)
{
    /* Mix two rand() draws with the tick — same non-crypto quality as before. */
    return ((uint32_t)rand() << 17) ^ ((uint32_t)rand() << 3) ^ (uint32_t)HAL_GetTick();
}

uint32_t m1_rng_get(void)
{
    if (s_state == 0)
        rng_hw_init();

    if (s_state == 1) {
        for (volatile uint32_t i = 0; i < RNG_SPIN_MAX; i++) {
            uint32_t sr = RNG->SR;
            if (sr & (RNG_SR_SEIS | RNG_SR_CEIS)) { s_state = 2; break; }
            if (sr & RNG_SR_DRDY) {
                /* Atomically re-check DRDY and consume DR so a concurrent caller
                 * can't leave us reading a stale/zero word. */
                uint32_t v = 0;
                __disable_irq();
                if (RNG->SR & RNG_SR_DRDY) v = RNG->DR;
                __enable_irq();
                if (v) return v;   /* lost the race -> DRDY cleared; try again */
            }
        }
        s_state = 2;
    }

    return sw_rand();
}
