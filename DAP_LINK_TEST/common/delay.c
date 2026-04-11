#include "delay.h"

#include "ti_msp_dl_config.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

static volatile uint32_t g_delay_tick_ms;

__attribute__((weak)) void delay_systick_callback(void)
{
}

void SysTick_Handler(void)
{
    g_delay_tick_ms++;
    delay_systick_callback();
}

void delay_cpu_cycles(uint32_t cycles)
{
    DL_Common_delayCycles(cycles);
}

void delay_us(uint32_t us)
{
    const uint32_t cycles_per_us = CPUCLK_FREQ / 1000000U;

    while (us-- > 0U) {
        DL_Common_delayCycles(cycles_per_us);
    }
}

void delay_ms(uint32_t ms)
{
    uint32_t start = g_delay_tick_ms;

    while ((uint32_t) (g_delay_tick_ms - start) < ms) {
        __WFI();
    }
}

uint32_t delay_get_ms(void)
{
    return g_delay_tick_ms;
}
