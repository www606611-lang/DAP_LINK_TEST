#include "board_gpio_irq.h"

#include "board_button.h"
#include "delay.h"
#include "encoder_input.h"
#include "ti_msp_dl_config.h"

#define BOARD_GPIO_IRQ_MAX_EVENTS 8U

static uint32_t board_gpiob_interrupt_pin_mask(void)
{
    return ENCODER_GPIOB_ENCODER_0_A_PIN |
        ENCODER_GPIOB_ENCODER_0_B_PIN |
        ENCODER_GPIOB_ENCODER_1_A_PIN |
        ENCODER_GPIOB_ENCODER_1_B_PIN |
        USER_BUTTON_PB21_PIN |
        POSITION_BUTTONS_SW2_PB4_PIN |
        POSITION_BUTTONS_SW1_PB5_PIN;
}

void BoardGpioIrq_Init(void)
{
    DL_GPIO_clearInterruptStatus(
        ENCODER_GPIOB_PORT, board_gpiob_interrupt_pin_mask());
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

void GROUP1_IRQHandler(void)
{
    uint32_t event_count;

    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) !=
        GPIO_MULTIPLE_GPIOB_INT_IIDX) {
        return;
    }

    for (event_count = 0U; event_count < BOARD_GPIO_IRQ_MAX_EVENTS;
         event_count++) {
        uint32_t interrupt_index = DL_GPIO_getPendingInterrupt(
            ENCODER_GPIOB_PORT);

        if (interrupt_index == DL_GPIO_IIDX_NO_INTR) {
            return;
        }
        if (!EncoderInput_OnGpioInterrupt(interrupt_index)) {
            (void) BoardButton_OnGpioInterrupt(
                interrupt_index, delay_get_ms());
        }
    }
}
