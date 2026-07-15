#include "board_motor_safe.h"

#include "motor_pwm.h"

static bool g_motor_armed;
static bool g_motor_high_impedance;
static board_motor_channel_t g_armed_channel;

static bool board_motor_safe_is_valid_channel(
    board_motor_channel_t channel)
{
    return ((uint32_t) channel < (uint32_t) BOARD_MOTOR_CHANNEL_COUNT);
}

static motor_pwm_channel_t board_motor_safe_get_pwm_channel(
    board_motor_channel_t channel)
{
    return (channel == BOARD_MOTOR_CHANNEL_A) ?
        MOTOR_PWM_CHANNEL_A : MOTOR_PWM_CHANNEL_B;
}

void BoardMotorSafe_Init(void)
{
    MotorPwm_Init();
    BoardMotorSafe_EmergencyStop();
}

void BoardMotorSafe_EmergencyStop(void)
{
    MotorPwm_DisableAll();
    g_motor_armed = false;
    g_motor_high_impedance = true;
    g_armed_channel = BOARD_MOTOR_CHANNEL_A;
}

bool BoardMotorSafe_Arm(board_motor_channel_t channel)
{
    if (!board_motor_safe_is_valid_channel(channel)) {
        return false;
    }

    MotorPwm_DisableAll();
    if (channel == BOARD_MOTOR_CHANNEL_BOTH) {
        if (!MotorPwm_Enable(MOTOR_PWM_CHANNEL_A) ||
            !MotorPwm_Enable(MOTOR_PWM_CHANNEL_B)) {
            BoardMotorSafe_EmergencyStop();
            return false;
        }
    } else if (!MotorPwm_Enable(
            board_motor_safe_get_pwm_channel(channel))) {
        BoardMotorSafe_EmergencyStop();
        return false;
    }

    g_motor_armed = true;
    g_motor_high_impedance = false;
    g_armed_channel = channel;
    return true;
}

bool BoardMotorSafe_IsArmed(void)
{
    return g_motor_armed;
}

bool BoardMotorSafe_IsHighImpedance(void)
{
    return g_motor_high_impedance;
}

board_motor_channel_t BoardMotorSafe_GetArmedChannel(void)
{
    return g_armed_channel;
}
