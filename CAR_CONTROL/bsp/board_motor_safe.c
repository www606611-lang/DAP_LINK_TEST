#include "board_motor_safe.h"

#include "ti_msp_dl_config.h"

static bool g_motor_high_impedance;

static uint32_t board_motor_safe_pin_mask(void)
{
    return MOTOR_SAFE_A_IN1_PA29_PIN |
        MOTOR_SAFE_A_IN2_PA30_PIN |
        MOTOR_SAFE_B_IN1_PA23_PIN |
        MOTOR_SAFE_B_IN2_PA24_PIN;
}

void BoardMotorSafe_Init(void)
{
    BoardMotorSafe_EmergencyStop();
}

void BoardMotorSafe_EmergencyStop(void)
{
    DL_GPIO_disableOutput(MOTOR_SAFE_PORT, board_motor_safe_pin_mask());
    DL_GPIO_initDigitalInput(MOTOR_SAFE_A_IN1_PA29_IOMUX);
    DL_GPIO_initDigitalInput(MOTOR_SAFE_A_IN2_PA30_IOMUX);
    DL_GPIO_initDigitalInput(MOTOR_SAFE_B_IN1_PA23_IOMUX);
    DL_GPIO_initDigitalInput(MOTOR_SAFE_B_IN2_PA24_IOMUX);
    g_motor_high_impedance = true;
}

bool BoardMotorSafe_IsArmed(void)
{
    return false;
}

bool BoardMotorSafe_IsHighImpedance(void)
{
    return g_motor_high_impedance;
}
