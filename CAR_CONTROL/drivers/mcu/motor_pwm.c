#include "motor_pwm.h"

#include "ti_msp_dl_config.h"

typedef struct {
    GPTIMER_Regs *timer;
    GPIO_Regs *port;
    uint32_t in1_pin;
    uint32_t in2_pin;
    uint32_t in1_iomux;
    uint32_t in2_iomux;
    uint32_t in1_iomux_func;
    uint32_t in2_iomux_func;
    DL_TIMER_CC_INDEX in1_index;
    DL_TIMER_CC_INDEX in2_index;
} motor_pwm_hw_t;

static bool g_enabled[MOTOR_PWM_CHANNEL_COUNT];

static bool motor_pwm_is_valid_channel(motor_pwm_channel_t channel);
static motor_pwm_hw_t motor_pwm_get_hw(motor_pwm_channel_t channel);
static void motor_pwm_force_low(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel);
static void motor_pwm_apply_duty(GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX channel, uint16_t duty_permille);

void MotorPwm_Init(void)
{
    g_enabled[MOTOR_PWM_CHANNEL_A] = false;
    g_enabled[MOTOR_PWM_CHANNEL_B] = false;
    MotorPwm_DisableAll();
}

bool MotorPwm_Enable(motor_pwm_channel_t channel)
{
    motor_pwm_hw_t hw;

    if (!motor_pwm_is_valid_channel(channel)) {
        return false;
    }
    if (g_enabled[channel]) {
        return true;
    }

    hw = motor_pwm_get_hw(channel);
    motor_pwm_force_low(hw.timer, hw.in1_index);
    motor_pwm_force_low(hw.timer, hw.in2_index);
    DL_GPIO_initPeripheralOutputFunction(hw.in1_iomux, hw.in1_iomux_func);
    DL_GPIO_initPeripheralOutputFunction(hw.in2_iomux, hw.in2_iomux_func);
    DL_GPIO_enableOutput(hw.port, hw.in1_pin | hw.in2_pin);
    DL_TimerG_startCounter(hw.timer);
    g_enabled[channel] = true;
    return true;
}

void MotorPwm_Disable(motor_pwm_channel_t channel)
{
    motor_pwm_hw_t hw;

    if (!motor_pwm_is_valid_channel(channel)) {
        return;
    }

    hw = motor_pwm_get_hw(channel);
    motor_pwm_force_low(hw.timer, hw.in1_index);
    motor_pwm_force_low(hw.timer, hw.in2_index);
    DL_TimerG_stopCounter(hw.timer);
    DL_GPIO_disableOutput(hw.port, hw.in1_pin | hw.in2_pin);
    DL_GPIO_initDigitalInput(hw.in1_iomux);
    DL_GPIO_initDigitalInput(hw.in2_iomux);
    g_enabled[channel] = false;
}

void MotorPwm_DisableAll(void)
{
    MotorPwm_Disable(MOTOR_PWM_CHANNEL_A);
    MotorPwm_Disable(MOTOR_PWM_CHANNEL_B);
}

bool MotorPwm_SetDuty(motor_pwm_channel_t channel,
    uint16_t in1_permille, uint16_t in2_permille)
{
    motor_pwm_hw_t hw;

    if ((!motor_pwm_is_valid_channel(channel)) ||
        (!g_enabled[channel]) ||
        (in1_permille > MOTOR_PWM_DUTY_MAX) ||
        (in2_permille > MOTOR_PWM_DUTY_MAX)) {
        return false;
    }

    hw = motor_pwm_get_hw(channel);
    motor_pwm_apply_duty(hw.timer, hw.in1_index, in1_permille);
    motor_pwm_apply_duty(hw.timer, hw.in2_index, in2_permille);
    return true;
}

bool MotorPwm_IsEnabled(motor_pwm_channel_t channel)
{
    if (!motor_pwm_is_valid_channel(channel)) {
        return false;
    }
    return g_enabled[channel];
}

static bool motor_pwm_is_valid_channel(motor_pwm_channel_t channel)
{
    return ((uint32_t) channel < (uint32_t) MOTOR_PWM_CHANNEL_COUNT);
}

static motor_pwm_hw_t motor_pwm_get_hw(motor_pwm_channel_t channel)
{
    motor_pwm_hw_t hw;

    if (channel == MOTOR_PWM_CHANNEL_A) {
        hw.timer = MOTOR_A_PWM_INST;
        hw.port = GPIO_MOTOR_A_PWM_C0_PORT;
        hw.in1_pin = GPIO_MOTOR_A_PWM_C0_PIN;
        hw.in2_pin = GPIO_MOTOR_A_PWM_C1_PIN;
        hw.in1_iomux = GPIO_MOTOR_A_PWM_C0_IOMUX;
        hw.in2_iomux = GPIO_MOTOR_A_PWM_C1_IOMUX;
        hw.in1_iomux_func = GPIO_MOTOR_A_PWM_C0_IOMUX_FUNC;
        hw.in2_iomux_func = GPIO_MOTOR_A_PWM_C1_IOMUX_FUNC;
        hw.in1_index = GPIO_MOTOR_A_PWM_C0_IDX;
        hw.in2_index = GPIO_MOTOR_A_PWM_C1_IDX;
    } else {
        hw.timer = MOTOR_B_PWM_INST;
        hw.port = GPIO_MOTOR_B_PWM_C0_PORT;
        hw.in1_pin = GPIO_MOTOR_B_PWM_C0_PIN;
        hw.in2_pin = GPIO_MOTOR_B_PWM_C1_PIN;
        hw.in1_iomux = GPIO_MOTOR_B_PWM_C0_IOMUX;
        hw.in2_iomux = GPIO_MOTOR_B_PWM_C1_IOMUX;
        hw.in1_iomux_func = GPIO_MOTOR_B_PWM_C0_IOMUX_FUNC;
        hw.in2_iomux_func = GPIO_MOTOR_B_PWM_C1_IOMUX_FUNC;
        hw.in1_index = GPIO_MOTOR_B_PWM_C0_IDX;
        hw.in2_index = GPIO_MOTOR_B_PWM_C1_IDX;
    }
    return hw;
}

static void motor_pwm_force_low(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel)
{
    DL_Timer_overrideCCPOut(timer, DL_TIMER_FORCE_OUT_LOW,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, channel);
}

static void motor_pwm_apply_duty(GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX channel, uint16_t duty_permille)
{
    uint32_t period;
    uint32_t high_ticks;
    uint32_t compare_value;

    if (duty_permille == 0U) {
        motor_pwm_force_low(timer, channel);
        return;
    }

    if (duty_permille == MOTOR_PWM_DUTY_MAX) {
        DL_Timer_overrideCCPOut(timer, DL_TIMER_FORCE_OUT_HIGH,
            DL_TIMER_FORCE_CMPL_OUT_DISABLED, channel);
        return;
    }

    period = DL_TimerG_getLoadValue(timer) + 1U;
    high_ticks = (period * duty_permille) / MOTOR_PWM_DUTY_MAX;
    compare_value = period - high_ticks;
    DL_TimerG_setCaptureCompareValue(timer, compare_value, channel);
    DL_Timer_overrideCCPOut(timer, DL_TIMER_FORCE_OUT_DISABLED,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, channel);
}
