#include "encoder_speed_control.h"

#include "encoder_motor_pid.h"
#include "encoder.h"
#include "motor.h"

#include <stdbool.h>
#include <stddef.h>

#define SPEED_CONTROL_TARGET_MIN_PPS      0.0f
#define SPEED_CONTROL_TARGET_MAX_PPS      3000.0f
#define SPEED_CONTROL_LEFT_KP             0.08f
#define SPEED_CONTROL_LEFT_KI             0.06f
#define SPEED_CONTROL_LEFT_KD             0.0f
#define SPEED_CONTROL_RIGHT_KP            0.06f
#define SPEED_CONTROL_RIGHT_KI            0.11f
#define SPEED_CONTROL_RIGHT_KD            0.0f
#define SPEED_CONTROL_LEFT_FEEDFORWARD    0.0f
#define SPEED_CONTROL_RIGHT_FEEDFORWARD   0.0f
#define SPEED_CONTROL_LEFT_I_LIMIT        3200.0f
#define SPEED_CONTROL_RIGHT_I_LIMIT       4000.0f
#define SPEED_CONTROL_DEADBAND            12.0f
#define SPEED_CONTROL_PWM_LIMIT           330.0f
#define SPEED_CONTROL_FEEDFORWARD_REF_PPS 1800.0f

static encoder_motor_pid_t g_left_speed_loop;
static encoder_motor_pid_t g_right_speed_loop;
static float g_left_target_pps;
static float g_right_target_pps;
static bool g_initialized;
static bool g_enabled;

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
    } else {
        EncoderMotorPID_SetSpeedTunings(loop, SPEED_CONTROL_RIGHT_KP,
            SPEED_CONTROL_RIGHT_KI, SPEED_CONTROL_RIGHT_KD);
    }

    EncoderMotorPID_SetSpeedFeedforwardPwm(loop,
        (motor_id == MOTOR_LEFT) ? SPEED_CONTROL_LEFT_FEEDFORWARD :
            SPEED_CONTROL_RIGHT_FEEDFORWARD);
    EncoderMotorPID_SetSpeedFeedforwardReferencePps(
        loop, SPEED_CONTROL_FEEDFORWARD_REF_PPS);
    EncoderMotorPID_SetSpeedOutputLimits(
        loop, -SPEED_CONTROL_PWM_LIMIT, SPEED_CONTROL_PWM_LIMIT);
    EncoderMotorPID_SetSpeedIntegralLimits(loop,
        (motor_id == MOTOR_LEFT) ? -SPEED_CONTROL_LEFT_I_LIMIT :
            -SPEED_CONTROL_RIGHT_I_LIMIT,
        (motor_id == MOTOR_LEFT) ? SPEED_CONTROL_LEFT_I_LIMIT :
            SPEED_CONTROL_RIGHT_I_LIMIT);
    EncoderMotorPID_SetSpeedDeadband(loop, SPEED_CONTROL_DEADBAND);
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
