#include "motor.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

typedef struct {
    motor_decay_t decay;
    bool inverted;
    int pwm;
} motor_state_t;

static motor_state_t g_motors[MOTOR_ID_COUNT];

static bool motor_is_valid_id(motor_id_t id);
static int motor_limit(int pwm);
static void motor_set(motor_id_t id, int pwm);
static void motor_stop(motor_id_t id);
static void motor_brake(motor_id_t id);
static void motor_set_decay(motor_id_t id, motor_decay_t decay);
static void motor_set_inverted(motor_id_t id, bool inverted);
static int motor_get_pwm(motor_id_t id);
static GPTIMER_Regs *motor_get_timer(motor_id_t id);
static DL_TIMER_CC_INDEX motor_get_in1_idx(motor_id_t id);
static DL_TIMER_CC_INDEX motor_get_in2_idx(motor_id_t id);
static void motor_apply_hw(motor_id_t id, int pwm);
static void motor_set_channel_duty(
    motor_id_t id, DL_TIMER_CC_INDEX channel, uint16_t duty_permille);
static uint32_t motor_get_pwm_period(motor_id_t id);

void Motor_Init(void)
{
    for (uint32_t i = 0U; i < (uint32_t) MOTOR_ID_COUNT; i++) {
        motor_id_t id = (motor_id_t) i;

        g_motors[id].decay = MOTOR_DECAY_SLOW;
        g_motors[id].inverted = false;
        motor_stop(id);
        DL_TimerG_startCounter(motor_get_timer(id));
    }
}

void Motor_Set(int pwm)
{
    Motor_SetLeft(pwm);
}

void Motor_SetLeft(int pwm)
{
    motor_set(MOTOR_LEFT, pwm);
}

void Motor_SetRight(int pwm)
{
    motor_set(MOTOR_RIGHT, pwm);
}

void Motor_Stop(void)
{
    Motor_StopLeft();
    Motor_StopRight();
}

void Motor_StopLeft(void)
{
    motor_stop(MOTOR_LEFT);
}

void Motor_StopRight(void)
{
    motor_stop(MOTOR_RIGHT);
}

void Motor_Brake(void)
{
    Motor_BrakeLeft();
    Motor_BrakeRight();
}

void Motor_BrakeLeft(void)
{
    motor_brake(MOTOR_LEFT);
}

void Motor_BrakeRight(void)
{
    motor_brake(MOTOR_RIGHT);
}

void Motor_SetDecay(motor_decay_t decay)
{
    Motor_SetLeftDecay(decay);
    Motor_SetRightDecay(decay);
}

void Motor_SetLeftDecay(motor_decay_t decay)
{
    motor_set_decay(MOTOR_LEFT, decay);
}

void Motor_SetRightDecay(motor_decay_t decay)
{
    motor_set_decay(MOTOR_RIGHT, decay);
}

void Motor_SetInverted(bool inverted)
{
    Motor_SetLeftInverted(inverted);
}

void Motor_SetLeftInverted(bool inverted)
{
    motor_set_inverted(MOTOR_LEFT, inverted);
}

void Motor_SetRightInverted(bool inverted)
{
    motor_set_inverted(MOTOR_RIGHT, inverted);
}

int Motor_GetPwm(void)
{
    return Motor_GetLeftPwm();
}

int Motor_GetLeftPwm(void)
{
    return motor_get_pwm(MOTOR_LEFT);
}

int Motor_GetRightPwm(void)
{
    return motor_get_pwm(MOTOR_RIGHT);
}

static bool motor_is_valid_id(motor_id_t id)
{
    return ((uint32_t) id < (uint32_t) MOTOR_ID_COUNT);
}

static int motor_limit(int pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }
    if (pwm < -MOTOR_PWM_MAX) {
        return -MOTOR_PWM_MAX;
    }
    return pwm;
}

static void motor_set(motor_id_t id, int pwm)
{
    if (!motor_is_valid_id(id)) {
        return;
    }

    g_motors[id].pwm = motor_limit(pwm);

    if (g_motors[id].inverted) {
        motor_apply_hw(id, -g_motors[id].pwm);
    } else {
        motor_apply_hw(id, g_motors[id].pwm);
    }
}

static void motor_stop(motor_id_t id)
{
    if (!motor_is_valid_id(id)) {
        return;
    }

    motor_set_channel_duty(id, motor_get_in1_idx(id), 0U);
    motor_set_channel_duty(id, motor_get_in2_idx(id), 0U);
    g_motors[id].pwm = 0;
}

static void motor_brake(motor_id_t id)
{
    if (!motor_is_valid_id(id)) {
        return;
    }

    motor_set_channel_duty(id, motor_get_in1_idx(id), MOTOR_PWM_MAX);
    motor_set_channel_duty(id, motor_get_in2_idx(id), MOTOR_PWM_MAX);
    g_motors[id].pwm = 0;
}

static void motor_set_decay(motor_id_t id, motor_decay_t decay)
{
    if ((!motor_is_valid_id(id)) ||
        ((decay != MOTOR_DECAY_SLOW) && (decay != MOTOR_DECAY_FAST))) {
        return;
    }

    g_motors[id].decay = decay;
    motor_set(id, g_motors[id].pwm);
}

static void motor_set_inverted(motor_id_t id, bool inverted)
{
    if (!motor_is_valid_id(id)) {
        return;
    }

    g_motors[id].inverted = inverted;
    motor_set(id, g_motors[id].pwm);
}

static int motor_get_pwm(motor_id_t id)
{
    if (!motor_is_valid_id(id)) {
        return 0;
    }

    return g_motors[id].pwm;
}

static GPTIMER_Regs *motor_get_timer(motor_id_t id)
{
    return (id == MOTOR_RIGHT) ? motorR_INST : motorL_INST;
}

static DL_TIMER_CC_INDEX motor_get_in1_idx(motor_id_t id)
{
    return (id == MOTOR_RIGHT) ? GPIO_motorR_C0_IDX : GPIO_motorL_C0_IDX;
}

static DL_TIMER_CC_INDEX motor_get_in2_idx(motor_id_t id)
{
    return (id == MOTOR_RIGHT) ? GPIO_motorR_C1_IDX : GPIO_motorL_C1_IDX;
}

static void motor_apply_hw(motor_id_t id, int pwm)
{
    uint16_t duty;
    uint16_t brake_duty;

    if (pwm == 0) {
        motor_set_channel_duty(id, motor_get_in1_idx(id), 0U);
        motor_set_channel_duty(id, motor_get_in2_idx(id), 0U);
        return;
    }

    duty = (uint16_t) ((pwm > 0) ? pwm : -pwm);

    if (g_motors[id].decay == MOTOR_DECAY_SLOW) {
        brake_duty = (uint16_t) (MOTOR_PWM_MAX - duty);

        if (pwm > 0) {
            motor_set_channel_duty(id, motor_get_in1_idx(id), MOTOR_PWM_MAX);
            motor_set_channel_duty(id, motor_get_in2_idx(id), brake_duty);
        } else {
            motor_set_channel_duty(id, motor_get_in1_idx(id), brake_duty);
            motor_set_channel_duty(id, motor_get_in2_idx(id), MOTOR_PWM_MAX);
        }
    } else {
        if (pwm > 0) {
            motor_set_channel_duty(id, motor_get_in1_idx(id), duty);
            motor_set_channel_duty(id, motor_get_in2_idx(id), 0U);
        } else {
            motor_set_channel_duty(id, motor_get_in1_idx(id), 0U);
            motor_set_channel_duty(id, motor_get_in2_idx(id), duty);
        }
    }
}

static void motor_set_channel_duty(
    motor_id_t id, DL_TIMER_CC_INDEX channel, uint16_t duty_permille)
{
    uint32_t compare_value;
    uint32_t high_ticks;
    uint32_t period = motor_get_pwm_period(id);

    if (duty_permille > MOTOR_PWM_MAX) {
        duty_permille = MOTOR_PWM_MAX;
    }

    high_ticks = (period * duty_permille) / MOTOR_PWM_MAX;
    compare_value = period - high_ticks;

    DL_TimerG_setCaptureCompareValue(motor_get_timer(id), compare_value,
        channel);
}

static uint32_t motor_get_pwm_period(motor_id_t id)
{
    return DL_TimerG_getLoadValue(motor_get_timer(id)) + 1U;
}
