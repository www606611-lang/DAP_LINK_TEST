#include "yaw_angle_control.h"

#include "encoder_speed_control.h"
#include "icm20948.h"
#include "pid.h"

#include <stdbool.h>
#include <stddef.h>

#define YAW_CONTROL_KP                  18.0f
#define YAW_CONTROL_KI                  0.0f
#define YAW_CONTROL_KD                  0.0f
#define YAW_CONTROL_UPDATE_MS           20U
#define YAW_CONTROL_DEADBAND_DEG        1.0f
#define YAW_CONTROL_SOFT_ZONE_DEG       3.0f
#define YAW_CONTROL_MAX_TURN_SPEED_PPS  900.0f
#define YAW_CONTROL_MIN_TURN_SPEED_PPS  200.0f
#define YAW_CONTROL_INTEGRAL_LIMIT      60.0f
#define YAW_CONTROL_LEFT_FORWARD_MIN_PWM 145.0f
#define YAW_CONTROL_LEFT_REVERSE_MIN_PWM 165.0f
#define YAW_CONTROL_RIGHT_FORWARD_MIN_PWM 380.0f
#define YAW_CONTROL_RIGHT_REVERSE_MIN_PWM 300.0f
#define YAW_CONTROL_MIN_DRIVE_REF_PPS   400.0f
#define YAW_CONTROL_SIGN                1.0f

static pid_controller_t g_yaw_pid;
static yaw_angle_control_state_t g_yaw_state;
static uint32_t g_yaw_last_update_ms;
static bool g_yaw_initialized;

static float yaw_angle_control_wrap_deg(float angle);
static float yaw_angle_control_abs(float value);
static float yaw_angle_control_clamp(
    float value, float min_value, float max_value);
static void yaw_angle_control_apply_speed_targets(float turn_speed_pps);

void YawAngleControl_Init(uint32_t now_ms)
{
    PID_Init(&g_yaw_pid);
    PID_SetTunings(&g_yaw_pid, YAW_CONTROL_KP, YAW_CONTROL_KI,
        YAW_CONTROL_KD);
    PID_SetOutputLimits(&g_yaw_pid, -YAW_CONTROL_MAX_TURN_SPEED_PPS,
        YAW_CONTROL_MAX_TURN_SPEED_PPS);
    PID_SetIntegralLimits(&g_yaw_pid, -YAW_CONTROL_INTEGRAL_LIMIT,
        YAW_CONTROL_INTEGRAL_LIMIT);
    PID_SetDeadband(&g_yaw_pid, YAW_CONTROL_DEADBAND_DEG);

    EncoderSpeedControl_Init(now_ms);
    EncoderSpeedControl_SetDirectionalMinDrivePwm(
        YAW_CONTROL_LEFT_FORWARD_MIN_PWM, YAW_CONTROL_LEFT_REVERSE_MIN_PWM,
        YAW_CONTROL_RIGHT_FORWARD_MIN_PWM, YAW_CONTROL_RIGHT_REVERSE_MIN_PWM,
        YAW_CONTROL_MIN_DRIVE_REF_PPS);

    g_yaw_state.target_yaw_deg = 0.0f;
    g_yaw_state.current_yaw_deg = 0.0f;
    g_yaw_state.error_deg = 0.0f;
    g_yaw_state.turn_speed_target_pps = 0.0f;
    g_yaw_state.left_speed_target_pps = 0.0f;
    g_yaw_state.right_speed_target_pps = 0.0f;
    g_yaw_state.imu_ready = false;
    g_yaw_state.enabled = false;
    g_yaw_last_update_ms = now_ms;
    g_yaw_initialized = true;
}

void YawAngleControl_Task(uint32_t now_ms)
{
    ICM20948_Angle_t angle;
    uint32_t elapsed_ms;
    float dt_s;
    float turn_speed_pps;
    float abs_error_deg;

    if (!g_yaw_initialized) {
        return;
    }

    EncoderSpeedControl_Task(now_ms);

    if (!g_yaw_state.enabled) {
        return;
    }

    elapsed_ms = now_ms - g_yaw_last_update_ms;
    if (elapsed_ms < YAW_CONTROL_UPDATE_MS) {
        return;
    }

    g_yaw_last_update_ms = now_ms;
    g_yaw_state.imu_ready = ICM20948_IsReady();
    if (!g_yaw_state.imu_ready) {
        yaw_angle_control_apply_speed_targets(0.0f);
        PID_Reset(&g_yaw_pid);
        return;
    }

    angle = ICM20948_GetAngle();
    g_yaw_state.current_yaw_deg = yaw_angle_control_wrap_deg(angle.yaw);
    g_yaw_state.error_deg = yaw_angle_control_wrap_deg(
        g_yaw_state.target_yaw_deg - g_yaw_state.current_yaw_deg);
    abs_error_deg = yaw_angle_control_abs(g_yaw_state.error_deg);

    dt_s = (float) elapsed_ms / 1000.0f;
    turn_speed_pps = PID_UpdateError(&g_yaw_pid, g_yaw_state.error_deg,
        g_yaw_state.current_yaw_deg, dt_s);
    turn_speed_pps = yaw_angle_control_clamp(turn_speed_pps,
        -YAW_CONTROL_MAX_TURN_SPEED_PPS, YAW_CONTROL_MAX_TURN_SPEED_PPS);
    if (g_yaw_state.error_deg == 0.0f) {
        turn_speed_pps = 0.0f;
    } else if ((abs_error_deg > YAW_CONTROL_SOFT_ZONE_DEG) &&
        (turn_speed_pps > 0.0f) &&
        (turn_speed_pps < YAW_CONTROL_MIN_TURN_SPEED_PPS)) {
        turn_speed_pps = YAW_CONTROL_MIN_TURN_SPEED_PPS;
    } else if ((abs_error_deg > YAW_CONTROL_SOFT_ZONE_DEG) &&
        (turn_speed_pps < 0.0f) &&
        (turn_speed_pps > -YAW_CONTROL_MIN_TURN_SPEED_PPS)) {
        turn_speed_pps = -YAW_CONTROL_MIN_TURN_SPEED_PPS;
    }

    yaw_angle_control_apply_speed_targets(turn_speed_pps);
}

void YawAngleControl_SetTargetDeg(float yaw_deg)
{
    g_yaw_state.target_yaw_deg = yaw_angle_control_wrap_deg(yaw_deg);
    g_yaw_state.enabled = true;
}

void YawAngleControl_AddTargetDeg(float delta_yaw_deg)
{
    YawAngleControl_SetTargetDeg(
        g_yaw_state.target_yaw_deg + delta_yaw_deg);
}

void YawAngleControl_HoldCurrentYaw(void)
{
    ICM20948_Angle_t angle = ICM20948_GetAngle();

    YawAngleControl_SetTargetDeg(angle.yaw);
}

void YawAngleControl_GetTargetDeg(float *yaw_deg)
{
    if (yaw_deg != NULL) {
        *yaw_deg = g_yaw_state.target_yaw_deg;
    }
}

void YawAngleControl_ZeroYaw(uint32_t now_ms)
{
    ICM20948_ResetAngle();
    PID_Reset(&g_yaw_pid);
    g_yaw_state.target_yaw_deg = 0.0f;
    g_yaw_state.current_yaw_deg = 0.0f;
    g_yaw_state.error_deg = 0.0f;
    g_yaw_last_update_ms = now_ms;
    yaw_angle_control_apply_speed_targets(0.0f);
}

void YawAngleControl_GetState(yaw_angle_control_state_t *state)
{
    if (state != NULL) {
        *state = g_yaw_state;
    }
}

void YawAngleControl_GetTunings(yaw_angle_control_pid_t *pid)
{
    if (pid != NULL) {
        pid->kp = g_yaw_pid.kp;
        pid->ki = g_yaw_pid.ki;
        pid->kd = g_yaw_pid.kd;
    }
}

void YawAngleControl_Stop(void)
{
    g_yaw_state.enabled = false;
    PID_Reset(&g_yaw_pid);
    yaw_angle_control_apply_speed_targets(0.0f);
    EncoderSpeedControl_Stop();
    EncoderSpeedControl_ClearMinDrivePwm();
}

static float yaw_angle_control_wrap_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float yaw_angle_control_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float yaw_angle_control_clamp(
    float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void yaw_angle_control_apply_speed_targets(float turn_speed_pps)
{
    g_yaw_state.turn_speed_target_pps = turn_speed_pps;
    g_yaw_state.left_speed_target_pps = -turn_speed_pps * YAW_CONTROL_SIGN;
    g_yaw_state.right_speed_target_pps = turn_speed_pps * YAW_CONTROL_SIGN;

    EncoderSpeedControl_SetTargetPps(g_yaw_state.left_speed_target_pps,
        g_yaw_state.right_speed_target_pps);
}
