#include "board_startup.h"

#include "motor_pwm.h"
#include "ti_msp_dl_config.h"

static void board_startup_force_motor_gpio_low(void)
{
    const uint32_t motor_pins = GPIO_MOTOR_A_PWM_C0_PIN |
        GPIO_MOTOR_A_PWM_C1_PIN | GPIO_MOTOR_B_PWM_C0_PIN |
        GPIO_MOTOR_B_PWM_C1_PIN;

    DL_GPIO_clearPins(GPIO_MOTOR_A_PWM_C0_PORT, motor_pins);
    DL_GPIO_initDigitalOutput(GPIO_MOTOR_A_PWM_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_MOTOR_A_PWM_C1_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_MOTOR_B_PWM_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_MOTOR_B_PWM_C1_IOMUX);
    DL_GPIO_enableOutput(GPIO_MOTOR_A_PWM_C0_PORT, motor_pins);
}

void BoardStartup_Init(void)
{
    SYSCFG_DL_initPower();
    board_startup_force_motor_gpio_low();
    SYSCFG_DL_GPIO_init();
    board_startup_force_motor_gpio_low();

    /* Keep this list aligned with the generated SYSCFG_DL_init(). */
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_MOTOR_A_PWM_init();
    SYSCFG_DL_MOTOR_B_PWM_init();
    MotorPwm_DisableAll();
    SYSCFG_DL_I2C_0_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_BLUETOOTH_UART_init();
    SYSCFG_DL_CHASSIS_RADIO_UART_init();
    SYSCFG_DL_SPI_LCD_init();
    SYSCFG_DL_DMA_init();
    SYSCFG_DL_SYSTICK_init();
    SYSCFG_DL_APP_WWDT_init();
}
