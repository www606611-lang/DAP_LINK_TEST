#include "timer.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t g_timer0_counter_ms;

void timer_common_init(void)
{
    NVIC_EnableIRQ(TIMER_0_counter_INST_INT_IRQN);
}

uint32_t timer_common_get_ms(void)
{
    return g_timer0_counter_ms;
}

void TIMER_0_counter_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_counter_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            g_timer0_counter_ms++;
            break;
        default:
            break;
    }
}
