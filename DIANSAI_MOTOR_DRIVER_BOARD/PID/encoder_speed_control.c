#include "encoder_speed_control.h"

#include "encoder_motor_pid.h"
#include "encoder.h"
#include "motor.h"

#include <stdbool.h>
#include <stddef.h>

#define SPEED_CONTROL_TARGET_MIN_PPS           -6000.0f
#define SPEED_CONTROL_TARGET_MAX_PPS            6000.0f
#define SPEED_CONTROL_LEFT_KP                   0.072f
#define SPEED_CONTROL_LEFT_KI                   0.095f
#define SPEED_CONTROL_LEFT_KD                   0.001f
#define SPEED_CONTROL_RIGHT_KP                  0.13f
#define SPEED_CONTROL_RIGHT_KI                  0.48f
#define SPEED_CONTROL_RIGHT_KD                  0.001f
#define SPEED_CONTROL_LEFT_FEEDFORWARD          0.0f
#define SPEED_CONTROL_RIGHT_FEEDFORWARD         0.0f
#define SPEED_CONTROL_LEFT_I_LIMIT              5000.0f
#define SPEED_CONTROL_RIGHT_I_LIMIT             2500.0f
#define SPEED_CONTROL_DEADBAND                  12.0f
#define SPEED_CONTROL_PWM_LIMIT                 1000.0f
#define SPEED_CONTROL_LEFT_FEEDFORWARD_REF_PPS  1800.0f
#define SPEED_CONTROL_RIGHT_FEEDFORWARD_REF_PPS 0.0f
#define SPEED_CONTROL_LEFT_MIN_DRIVE_PPS        0.0f
#define SPEED_CONTROL_LEFT_MIN_DRIVE_REF_PPS    600.0f
#define SPEED_CONTROL_RIGHT_MIN_DRIVE_PPS       0.0f
#define SPEED_CONTROL_RIGHT_MIN_DRIVE_REF_PPS   3000.0f

static encoder_motor_pid_t g_left_speed_loop;
static encoder_motor_pid_t g_right_speed_loop;
static float g_left_target_pps;
static float g_right_target_pps;
static bool g_initialized;
static bool g_enabled;

static encoder_motor_pid_t *encoder_speed_control_get_loop(encoder_id_t id);
static void encoder_speed_control_configure_loop(
    encoder_motor_pid_t *loop, motor_id_t motor_id, encoder_id_t encoder_id,
    uint32_t now_ms);
static float encoder_speed_control_clamp_target(float target_speed_pps);

void EncoderSpeedControl_Init(uint32_t now_ms)
{
    Encoder_SetInverted(ENCODER_LEFT, true);
    Encoder_ResetAll();
    Motor_SetRightInverted(true);

    encoder_speed_control_configure_loop(
        &g_left_speed_loop, MOTOR_LEFT, ENCODER_LEFT, now_ms);
    encoder_speed_control_configure_loop(
        &g_right_speed_loop, MOTOR_RIGHT, ENCODER_RIGHT, now_ms);

    g_left_target_pps = 0.0f;
    g_right_target_pps = 0.0f;
    g_initialized = true;
    g_enabled = false;
    Motor_Stop();
}

void EncoderSpeedControl_Task(uint32_t now_ms)
{
    if (!g_initialized || !g_enabled) {
        return;
    }

    (void) EncoderMotorPID_Update(&g_left_speed_loop, now_ms);
    (void) EncoderMotorPID_Update(&g_right_speed_loop, now_ms);
}

bool EncoderSpeedControl_IsInitialized(void)
{
    return g_initialized;
}

void EncoderSpeedControl_SetTargetPps(
    float left_speed_pps, float right_speed_pps)
{
    g_left_target_pps = encoder_speed_control_clamp_target(left_speed_pps);
    g_right_target_pps = encoder_speed_control_clamp_target(right_speed_pps);

    if (!g_initialized) {
        return;
    }

    EncoderMotorPID_SetTargetSpeedPps(&g_left_speed_loop, g_left_target_pps);
    EncoderMotorPID_SetTargetSpeedPps(&g_right_speed_loop, g_right_target_pps);
    g_enabled = ((g_left_target_pps != 0.0f) || (g_right_target_pps != 0.0f));

    if (!g_enabled) {
        EncoderSpeedControl_Stop();
    }
}

void EncoderSpeedControl_GetTargetPps(
    float *left_speed_pps, float *right_speed_pps)
{
    if (left_speed_pps != NULL) {
        *left_speed_pps = g_left_target_pps;
    }
    if (right_speed_pps != NULL) {
        *right_speed_pps = g_right_target_pps;
    }
}

void EncoderSpeedControl_GetPwmCommand(
    float *left_pwm_command, float *right_pwm_command)
{
    if (left_pwm_command != NULL) {
        *left_pwm_command = EncoderMotorPID_GetPwmCommand(
            &g_left_speed_loop);
    }
    if (right_pwm_command != NULL) {
        *right_pwm_command = EncoderMotorPID_GetPwmCommand(
            &g_right_speed_loop);
    }
}

void EncoderSpeedControl_SetSpeedTunings(
    encoder_id_t id, float kp, float ki, float kd)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedTunings(loop, kp, ki, kd);
}

void EncoderSpeedControl_SetSpeedOutputLimits(
    encoder_id_t id, float output_min, float output_max)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedOutputLimits(loop, output_min, output_max);
}

void EncoderSpeedControl_SetSpeedIntegralLimits(
    encoder_id_t id, float integral_min, float integral_max)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedIntegralLimits(loop, integral_min, integral_max);
}

void EncoderSpeedControl_SetSpeedDeadband(
    encoder_id_t id, float deadband)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedDeadband(loop, deadband);
}

void EncoderSpeedControl_SetSpeedFeedforwardPwm(
    encoder_id_t id, float feedforward_pwm)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedFeedforwardPwm(loop, feedforward_pwm);
}

void EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
    encoder_id_t id, float reference_pps)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedFeedforwardReferencePps(loop, reference_pps);
}

void EncoderSpeedControl_SetSpeedMinDriveConfig(
    encoder_id_t id, float forward_pwm, float reverse_pwm,
    float reference_pps)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
        loop, forward_pwm, reverse_pwm);
    EncoderMotorPID_SetSpeedMinDriveReferencePps(loop, reference_pps);
}

void EncoderSpeedControl_SetMinDrivePwm(
    float left_pwm, float right_pwm, float reference_pps)
{
    EncoderSpeedControl_SetDirectionalMinDrivePwm(left_pwm, left_pwm,
        right_pwm, right_pwm, reference_pps);
}

void EncoderSpeedControl_SetDirectionalMinDrivePwm(
    float left_forward_pwm, float left_reverse_pwm,
    float right_forward_pwm, float right_reverse_pwm,
    float reference_pps)
{
    if (!g_initialized) {
        return;
    }

    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
        &g_left_speed_loop, left_forward_pwm, left_reverse_pwm);
    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
        &g_right_speed_loop, right_forward_pwm, right_reverse_pwm);
    EncoderMotorPID_SetSpeedMinDriveReferencePps(
        &g_left_speed_loop, reference_pps);
    EncoderMotorPID_SetSpeedMinDriveReferencePps(
        &g_right_speed_loop, reference_pps);
}

void EncoderSpeedControl_ClearMinDrivePwm(void)
{
    EncoderSpeedControl_SetDirectionalMinDrivePwm(
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void EncoderSpeedControl_SetOutputLimits(float left_pwm_limit,
    float right_pwm_limit)
{
    if (!g_initialized) {
        return;
    }

    if (left_pwm_limit < 0.0f) {
        left_pwm_limit = -left_pwm_limit;
    }
    if (right_pwm_limit < 0.0f) {
        right_pwm_limit = -right_pwm_limit;
    }
    if (left_pwm_limit > (float) MOTOR_PWM_MAX) {
        left_pwm_limit = (float) MOTOR_PWM_MAX;
    }
    if (right_pwm_limit > (float) MOTOR_PWM_MAX) {
        right_pwm_limit = (float) MOTOR_PWM_MAX;
    }

    EncoderMotorPID_SetSpeedOutputLimits(
        &g_left_speed_loop, -left_pwm_limit, left_pwm_limit);
    EncoderMotorPID_SetSpeedOutputLimits(
        &g_right_speed_loop, -right_pwm_limit, right_pwm_limit);
}

void EncoderSpeedControl_RestoreDefaultOutputLimits(void)
{
    EncoderSpeedControl_SetOutputLimits(SPEED_CONTROL_PWM_LIMIT,
        SPEED_CONTROL_PWM_LIMIT);
}

void EncoderSpeedControl_GetSpeedTunings(
    encoder_speed_control_pid_t *left, encoder_speed_control_pid_t *right)
{
    if (left != NULL) {
        left->kp = g_left_speed_loop.speed_pid.kp;
        left->ki = g_left_speed_loop.speed_pid.ki;
        left->kd = g_left_speed_loop.speed_pid.kd;
    }
    if (right != NULL) {
        right->kp = g_right_speed_loop.speed_pid.kp;
        right->ki = g_right_speed_loop.speed_pid.ki;
        right->kd = g_right_speed_loop.speed_pid.kd;
    }
}

void EncoderSpeedControl_GetSpeedConfig(
    encoder_id_t id, encoder_speed_control_config_t *config)
{
    encoder_motor_pid_t *loop = encoder_speed_control_get_loop(id);

    if ((loop == NULL) || (config == NULL)) {
        return;
    }

    config->kp = loop->speed_pid.kp;
    config->ki = loop->speed_pid.ki;
    config->kd = loop->speed_pid.kd;
    config->output_min = loop->speed_pid.output_min;
    config->output_max = loop->speed_pid.output_max;
    config->integral_min = loop->speed_pid.integral_min;
    config->integral_max = loop->speed_pid.integral_max;
    config->deadband = loop->speed_pid.deadband;
    config->feedforward_pwm = loop->speed_feedforward_pwm;
    config->feedforward_reference_pps = loop->speed_feedforward_reference_pps;
    config->forward_min_drive_pwm = loop->speed_forward_min_drive_pwm;
    config->reverse_min_drive_pwm = loop->speed_reverse_min_drive_pwm;
    config->min_drive_reference_pps = loop->speed_min_drive_reference_pps;
}

void EncoderSpeedControl_Stop(void)
{
    g_left_target_pps = 0.0f;
    g_right_target_pps = 0.0f;
    g_enabled = false;

    if (g_initialized) {
        EncoderMotorPID_Stop(&g_left_speed_loop);
        EncoderMotorPID_Stop(&g_right_speed_loop);
    }
    Motor_Stop();
}

static void encoder_speed_control_configure_loop(
    encoder_motor_pid_t *loop, motor_id_t motor_id, encoder_id_t encoder_id,
    uint32_t now_ms)
{
    EncoderMotorPID_Init(loop, motor_id, encoder_id, now_ms);
    if (motor_id == MOTOR_LEFT) {
        EncoderMotorPID_SetSpeedTunings(loop, SPEED_CONTROL_LEFT_KP,
            SPEED_CONTROL_LEFT_KI, SPEED_CONTROL_LEFT_KD);
        EncoderMotorPID_SetSpeedFeedforwardPwm(loop,
            SPEED_CONTROL_LEFT_FEEDFORWARD);
        EncoderMotorPID_SetSpeedFeedforwardReferencePps(loop,
            SPEED_CONTROL_LEFT_FEEDFORWARD_REF_PPS);
        EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(loop,
            SPEED_CONTROL_LEFT_MIN_DRIVE_PPS,
            SPEED_CONTROL_LEFT_MIN_DRIVE_PPS);
        EncoderMotorPID_SetSpeedMinDriveReferencePps(loop,
            SPEED_CONTROL_LEFT_MIN_DRIVE_REF_PPS);
    } else {
        EncoderMotorPID_SetSpeedTunings(loop, SPEED_CONTROL_RIGHT_KP,
            SPEED_CONTROL_RIGHT_KI, SPEED_CONTROL_RIGHT_KD);
        EncoderMotorPID_SetSpeedFeedforwardPwm(loop,
            SPEED_CONTROL_RIGHT_FEEDFORWARD);
        EncoderMotorPID_SetSpeedFeedforwardReferencePps(loop,
            SPEED_CONTROL_RIGHT_FEEDFORWARD_REF_PPS);
        EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(loop,
            SPEED_CONTROL_RIGHT_MIN_DRIVE_PPS,
            SPEED_CONTROL_RIGHT_MIN_DRIVE_PPS);
        EncoderMotorPID_SetSpeedMinDriveReferencePps(loop,
            SPEED_CONTROL_RIGHT_MIN_DRIVE_REF_PPS);
    }
    EncoderMotorPID_SetSpeedOutputLimits(
        loop, -SPEED_CONTROL_PWM_LIMIT, SPEED_CONTROL_PWM_LIMIT);
    EncoderMotorPID_SetSpeedIntegralLimits(loop,
        (motor_id == MOTOR_LEFT) ? -SPEED_CONTROL_LEFT_I_LIMIT :
            -SPEED_CONTROL_RIGHT_I_LIMIT,
        (motor_id == MOTOR_LEFT) ? SPEED_CONTROL_LEFT_I_LIMIT :
            SPEED_CONTROL_RIGHT_I_LIMIT);
    EncoderMotorPID_SetSpeedDeadband(loop, SPEED_CONTROL_DEADBAND);
}

static encoder_motor_pid_t *encoder_speed_control_get_loop(encoder_id_t id)
{
    if (!g_initialized) {
        return NULL;
    }

    switch (id) {
        case ENCODER_LEFT:
            return &g_left_speed_loop;
        case ENCODER_RIGHT:
            return &g_right_speed_loop;
        default:
            return NULL;
    }
}

static float encoder_speed_control_clamp_target(float target_speed_pps)
{
    if (target_speed_pps < SPEED_CONTROL_TARGET_MIN_PPS) {
        return SPEED_CONTROL_TARGET_MIN_PPS;
    }
    if (target_speed_pps > SPEED_CONTROL_TARGET_MAX_PPS) {
        return SPEED_CONTROL_TARGET_MAX_PPS;
    }

    return target_speed_pps;
}
