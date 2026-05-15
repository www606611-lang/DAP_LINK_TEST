#include "encoder_position_control.h"

#include "encoder_motor_pid.h"
#include "encoder.h"
#include "motor.h"

#include <stdbool.h>
#include <stddef.h>

#define POSITION_CONTROL_LEFT_POSITION_KP       4.0f
#define POSITION_CONTROL_LEFT_POSITION_KI       0.0f
#define POSITION_CONTROL_LEFT_POSITION_KD       0.0f
#define POSITION_CONTROL_RIGHT_POSITION_KP      4.0f
#define POSITION_CONTROL_RIGHT_POSITION_KI      0.0f
#define POSITION_CONTROL_RIGHT_POSITION_KD      0.0f
#define POSITION_CONTROL_LEFT_SPEED_KP          0.08f
#define POSITION_CONTROL_LEFT_SPEED_KI          0.06f
#define POSITION_CONTROL_LEFT_SPEED_KD          0.0f
#define POSITION_CONTROL_RIGHT_SPEED_KP         0.06f
#define POSITION_CONTROL_RIGHT_SPEED_KI         0.11f
#define POSITION_CONTROL_RIGHT_SPEED_KD         0.0f
#define POSITION_CONTROL_LEFT_I_LIMIT           3200.0f
#define POSITION_CONTROL_RIGHT_I_LIMIT          4000.0f
#define POSITION_CONTROL_SPEED_DEADBAND         12.0f
#define POSITION_CONTROL_POSITION_DEADBAND      4.0f
#define POSITION_CONTROL_PWM_LIMIT              330.0f
#define POSITION_CONTROL_MAX_SPEED_PPS          1600.0f

static encoder_motor_pid_t g_left_position_loop;
static encoder_motor_pid_t g_right_position_loop;
static float g_left_target_count;
static float g_right_target_count;
static bool g_initialized;
static bool g_enabled;

static void encoder_position_control_configure_loop(
    encoder_motor_pid_t *loop, motor_id_t motor_id, encoder_id_t encoder_id,
    uint32_t now_ms);
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
        EncoderMotorPID_SetSpeedTunings(loop, POSITION_CONTROL_LEFT_SPEED_KP,
            POSITION_CONTROL_LEFT_SPEED_KI, POSITION_CONTROL_LEFT_SPEED_KD);
        EncoderMotorPID_SetSpeedIntegralLimits(loop,
            -POSITION_CONTROL_LEFT_I_LIMIT, POSITION_CONTROL_LEFT_I_LIMIT);
    } else {
        EncoderMotorPID_SetPositionTunings(loop,
            POSITION_CONTROL_RIGHT_POSITION_KP,
            POSITION_CONTROL_RIGHT_POSITION_KI,
            POSITION_CONTROL_RIGHT_POSITION_KD);
        EncoderMotorPID_SetSpeedTunings(loop, POSITION_CONTROL_RIGHT_SPEED_KP,
            POSITION_CONTROL_RIGHT_SPEED_KI, POSITION_CONTROL_RIGHT_SPEED_KD);
        EncoderMotorPID_SetSpeedIntegralLimits(loop,
            -POSITION_CONTROL_RIGHT_I_LIMIT, POSITION_CONTROL_RIGHT_I_LIMIT);
    }

    EncoderMotorPID_SetPositionOutputLimits(loop,
        -POSITION_CONTROL_MAX_SPEED_PPS, POSITION_CONTROL_MAX_SPEED_PPS);
    EncoderMotorPID_SetSpeedOutputLimits(loop,
        -POSITION_CONTROL_PWM_LIMIT, POSITION_CONTROL_PWM_LIMIT);
    EncoderMotorPID_SetPositionDeadband(loop, POSITION_CONTROL_POSITION_DEADBAND);
    EncoderMotorPID_SetSpeedDeadband(loop, POSITION_CONTROL_SPEED_DEADBAND);
}

static void encoder_position_control_apply_targets(void)
{
    EncoderMotorPID_SetTargetPositionCount(
        &g_left_position_loop, g_left_target_count);
    EncoderMotorPID_SetTargetPositionCount(
        &g_right_position_loop, g_right_target_count);
    g_enabled = true;
}
