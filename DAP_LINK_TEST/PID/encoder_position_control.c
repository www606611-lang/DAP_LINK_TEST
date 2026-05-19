#include "encoder_position_control.h"

#include "encoder_motor_pid.h"
#include "encoder.h"
#include "encoder_speed_control.h"
#include "motor.h"

#include <stdbool.h>
#include <stddef.h>

#define POSITION_CONTROL_LEFT_POSITION_KP       4.0f
#define POSITION_CONTROL_LEFT_POSITION_KI       0.0f
#define POSITION_CONTROL_LEFT_POSITION_KD       0.0f
#define POSITION_CONTROL_RIGHT_POSITION_KP      4.0f
#define POSITION_CONTROL_RIGHT_POSITION_KI      0.0f
#define POSITION_CONTROL_RIGHT_POSITION_KD      0.0f
#define POSITION_CONTROL_POSITION_DEADBAND      4.0f
#define POSITION_CONTROL_PWM_LIMIT              1000.0f
#define POSITION_CONTROL_MAX_SPEED_PPS          1600.0f

static encoder_motor_pid_t g_left_position_loop;
static encoder_motor_pid_t g_right_position_loop;
static float g_left_target_count;
static float g_right_target_count;
static bool g_initialized;
static bool g_enabled;

static encoder_motor_pid_t *encoder_position_control_get_loop(encoder_id_t id);
static void encoder_position_control_configure_loop(
    encoder_motor_pid_t *loop, motor_id_t motor_id, encoder_id_t encoder_id,
    uint32_t now_ms);
static void encoder_position_control_sync_speed_from_global(
    encoder_motor_pid_t *loop, encoder_id_t id);
static void encoder_position_control_apply_targets(void);

void EncoderPositionControl_Init(uint32_t now_ms)
{
    Encoder_SetInverted(ENCODER_LEFT, true);
    Encoder_ResetAll();
    Motor_SetRightInverted(true);

    encoder_position_control_configure_loop(
        &g_left_position_loop, MOTOR_LEFT, ENCODER_LEFT, now_ms);
    encoder_position_control_configure_loop(
        &g_right_position_loop, MOTOR_RIGHT, ENCODER_RIGHT, now_ms);

    g_left_target_count = 0.0f;
    g_right_target_count = 0.0f;
    g_initialized = true;
    g_enabled = false;
    Motor_Stop();
}

void EncoderPositionControl_Task(uint32_t now_ms)
{
    if (!g_initialized || !g_enabled) {
        return;
    }

    (void) EncoderMotorPID_Update(&g_left_position_loop, now_ms);
    (void) EncoderMotorPID_Update(&g_right_position_loop, now_ms);
}

void EncoderPositionControl_SetTargetCount(
    float left_count, float right_count)
{
    g_left_target_count = left_count;
    g_right_target_count = right_count;

    if (!g_initialized) {
        return;
    }

    encoder_position_control_apply_targets();
}

void EncoderPositionControl_AddTargetCount(
    float left_delta_count, float right_delta_count)
{
    EncoderPositionControl_SetTargetCount(
        g_left_target_count + left_delta_count,
        g_right_target_count + right_delta_count);
}

void EncoderPositionControl_GetTargetCount(
    float *left_count, float *right_count)
{
    if (left_count != NULL) {
        *left_count = g_left_target_count;
    }
    if (right_count != NULL) {
        *right_count = g_right_target_count;
    }
}

void EncoderPositionControl_GetCurrentCount(
    int32_t *left_count, int32_t *right_count)
{
    if (left_count != NULL) {
        *left_count = Encoder_GetCount(ENCODER_LEFT);
    }
    if (right_count != NULL) {
        *right_count = Encoder_GetCount(ENCODER_RIGHT);
    }
}

void EncoderPositionControl_SetPositionTunings(
    encoder_id_t id, float kp, float ki, float kd)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetPositionTunings(loop, kp, ki, kd);
}

void EncoderPositionControl_SetPositionOutputLimits(
    encoder_id_t id, float output_min, float output_max)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetPositionOutputLimits(loop, output_min, output_max);
}

void EncoderPositionControl_SetPositionIntegralLimits(
    encoder_id_t id, float integral_min, float integral_max)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetPositionIntegralLimits(loop, integral_min,
        integral_max);
}

void EncoderPositionControl_SetPositionDeadband(
    encoder_id_t id, float deadband)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetPositionDeadband(loop, deadband);
}

void EncoderPositionControl_SetSpeedTunings(
    encoder_id_t id, float kp, float ki, float kd)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedTunings(loop, kp, ki, kd);
}

void EncoderPositionControl_SetSpeedOutputLimits(
    encoder_id_t id, float output_min, float output_max)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedOutputLimits(loop, output_min, output_max);
}

void EncoderPositionControl_SetSpeedIntegralLimits(
    encoder_id_t id, float integral_min, float integral_max)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedIntegralLimits(loop, integral_min, integral_max);
}

void EncoderPositionControl_SetSpeedDeadband(
    encoder_id_t id, float deadband)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedDeadband(loop, deadband);
}

void EncoderPositionControl_SetSpeedFeedforwardPwm(
    encoder_id_t id, float feedforward_pwm)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedFeedforwardPwm(loop, feedforward_pwm);
}

void EncoderPositionControl_SetSpeedFeedforwardReferencePps(
    encoder_id_t id, float reference_pps)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedFeedforwardReferencePps(loop, reference_pps);
}

void EncoderPositionControl_SetSpeedMinDriveConfig(
    encoder_id_t id, float forward_pwm, float reverse_pwm,
    float reference_pps)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if (loop == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
        loop, forward_pwm, reverse_pwm);
    EncoderMotorPID_SetSpeedMinDriveReferencePps(loop, reference_pps);
}

void EncoderPositionControl_SyncSpeedFromCurrent(void)
{
    if (!g_initialized) {
        return;
    }

    encoder_position_control_sync_speed_from_global(
        &g_left_position_loop, ENCODER_LEFT);
    encoder_position_control_sync_speed_from_global(
        &g_right_position_loop, ENCODER_RIGHT);
}

void EncoderPositionControl_ZeroPosition(uint32_t now_ms)
{
    EncoderPositionControl_Stop();
    Encoder_ResetAll();
    EncoderMotorPID_Reset(&g_left_position_loop, now_ms);
    EncoderMotorPID_Reset(&g_right_position_loop, now_ms);
    g_left_target_count = 0.0f;
    g_right_target_count = 0.0f;
}

void EncoderPositionControl_GetState(
    encoder_position_control_state_t *left,
    encoder_position_control_state_t *right)
{
    encoder_snapshot_t snapshot;

    if (left != NULL) {
        if (EncoderMotorPID_GetSnapshot(&g_left_position_loop, &snapshot)) {
            left->count = snapshot.count;
            left->speed_pps = snapshot.speed_pps;
        } else {
            left->count = Encoder_GetCount(ENCODER_LEFT);
            left->speed_pps = Encoder_GetSpeedPps(ENCODER_LEFT);
        }
        left->cascade_speed_target_pps =
            EncoderMotorPID_GetSpeedTargetPps(&g_left_position_loop);
        left->pwm_command =
            EncoderMotorPID_GetPwmCommand(&g_left_position_loop);
    }

    if (right != NULL) {
        if (EncoderMotorPID_GetSnapshot(&g_right_position_loop, &snapshot)) {
            right->count = snapshot.count;
            right->speed_pps = snapshot.speed_pps;
        } else {
            right->count = Encoder_GetCount(ENCODER_RIGHT);
            right->speed_pps = Encoder_GetSpeedPps(ENCODER_RIGHT);
        }
        right->cascade_speed_target_pps =
            EncoderMotorPID_GetSpeedTargetPps(&g_right_position_loop);
        right->pwm_command =
            EncoderMotorPID_GetPwmCommand(&g_right_position_loop);
    }
}

void EncoderPositionControl_GetPositionTunings(
    encoder_position_control_pid_t *left,
    encoder_position_control_pid_t *right)
{
    if (left != NULL) {
        left->kp = g_left_position_loop.position_pid.kp;
        left->ki = g_left_position_loop.position_pid.ki;
        left->kd = g_left_position_loop.position_pid.kd;
    }
    if (right != NULL) {
        right->kp = g_right_position_loop.position_pid.kp;
        right->ki = g_right_position_loop.position_pid.ki;
        right->kd = g_right_position_loop.position_pid.kd;
    }
}

void EncoderPositionControl_GetSpeedTunings(
    encoder_position_control_pid_t *left,
    encoder_position_control_pid_t *right)
{
    if (left != NULL) {
        left->kp = g_left_position_loop.speed_pid.kp;
        left->ki = g_left_position_loop.speed_pid.ki;
        left->kd = g_left_position_loop.speed_pid.kd;
    }
    if (right != NULL) {
        right->kp = g_right_position_loop.speed_pid.kp;
        right->ki = g_right_position_loop.speed_pid.ki;
        right->kd = g_right_position_loop.speed_pid.kd;
    }
}

void EncoderPositionControl_GetPositionConfig(
    encoder_id_t id, encoder_position_control_pid_config_t *config)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

    if ((loop == NULL) || (config == NULL)) {
        return;
    }

    config->kp = loop->position_pid.kp;
    config->ki = loop->position_pid.ki;
    config->kd = loop->position_pid.kd;
    config->output_min = loop->position_pid.output_min;
    config->output_max = loop->position_pid.output_max;
    config->integral_min = loop->position_pid.integral_min;
    config->integral_max = loop->position_pid.integral_max;
    config->deadband = loop->position_pid.deadband;
}

void EncoderPositionControl_GetSpeedConfig(
    encoder_id_t id, encoder_position_control_pid_config_t *config)
{
    encoder_motor_pid_t *loop = encoder_position_control_get_loop(id);

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
}

void EncoderPositionControl_Stop(void)
{
    g_enabled = false;

    if (g_initialized) {
        EncoderMotorPID_Stop(&g_left_position_loop);
        EncoderMotorPID_Stop(&g_right_position_loop);
    }
    Motor_Stop();
}

static void encoder_position_control_configure_loop(
    encoder_motor_pid_t *loop, motor_id_t motor_id, encoder_id_t encoder_id,
    uint32_t now_ms)
{
    EncoderMotorPID_Init(loop, motor_id, encoder_id, now_ms);

    if (motor_id == MOTOR_LEFT) {
        EncoderMotorPID_SetPositionTunings(loop,
            POSITION_CONTROL_LEFT_POSITION_KP,
            POSITION_CONTROL_LEFT_POSITION_KI,
            POSITION_CONTROL_LEFT_POSITION_KD);
    } else {
        EncoderMotorPID_SetPositionTunings(loop,
            POSITION_CONTROL_RIGHT_POSITION_KP,
            POSITION_CONTROL_RIGHT_POSITION_KI,
            POSITION_CONTROL_RIGHT_POSITION_KD);
    }

    EncoderMotorPID_SetPositionOutputLimits(loop,
        -POSITION_CONTROL_MAX_SPEED_PPS, POSITION_CONTROL_MAX_SPEED_PPS);
    EncoderMotorPID_SetSpeedOutputLimits(loop,
        -POSITION_CONTROL_PWM_LIMIT, POSITION_CONTROL_PWM_LIMIT);
    EncoderMotorPID_SetPositionDeadband(loop, POSITION_CONTROL_POSITION_DEADBAND);
    encoder_position_control_sync_speed_from_global(
        loop, (motor_id == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT);
}

static void encoder_position_control_apply_targets(void)
{
    EncoderMotorPID_SetTargetPositionCount(
        &g_left_position_loop, g_left_target_count);
    EncoderMotorPID_SetTargetPositionCount(
        &g_right_position_loop, g_right_target_count);
    g_enabled = true;
}

static encoder_motor_pid_t *encoder_position_control_get_loop(encoder_id_t id)
{
    if (!g_initialized) {
        return NULL;
    }

    switch (id) {
        case ENCODER_LEFT:
            return &g_left_position_loop;
        case ENCODER_RIGHT:
            return &g_right_position_loop;
        default:
            return NULL;
    }
}

static void encoder_position_control_sync_speed_from_global(
    encoder_motor_pid_t *loop, encoder_id_t id)
{
    encoder_speed_control_config_t speed_config;

    if (loop == NULL) {
        return;
    }
    if (!EncoderSpeedControl_IsInitialized()) {
        return;
    }

    EncoderSpeedControl_GetSpeedConfig(id, &speed_config);

    EncoderMotorPID_SetSpeedTunings(loop,
        speed_config.kp, speed_config.ki, speed_config.kd);
    EncoderMotorPID_SetSpeedOutputLimits(loop,
        speed_config.output_min, speed_config.output_max);
    EncoderMotorPID_SetSpeedIntegralLimits(loop,
        speed_config.integral_min, speed_config.integral_max);
    EncoderMotorPID_SetSpeedDeadband(loop, speed_config.deadband);
    EncoderMotorPID_SetSpeedFeedforwardPwm(loop,
        speed_config.feedforward_pwm);
    EncoderMotorPID_SetSpeedFeedforwardReferencePps(loop,
        speed_config.feedforward_reference_pps);
    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(loop,
        speed_config.forward_min_drive_pwm,
        speed_config.reverse_min_drive_pwm);
    EncoderMotorPID_SetSpeedMinDriveReferencePps(loop,
        speed_config.min_drive_reference_pps);
}
