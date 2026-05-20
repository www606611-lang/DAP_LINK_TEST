#include "line_tracking_control.h"

#include "encoder.h"
#include "encoder_speed_control.h"
#include "line_sensor_i2c.h"
#include "pid.h"

#include <stddef.h>

#define LINE_TRACKING_UPDATE_MS             20U
#define LINE_TRACKING_DT_CLAMP_MS           40U
#define LINE_TRACKING_START_RAMP_MS         500U
#define LINE_TRACKING_BASE_SPEED_PPS        760.0f
#define LINE_TRACKING_MAX_BASE_SPEED_PPS    1800.0f
#define LINE_TRACKING_MAX_TURN_PPS          2000.0f
#define LINE_TRACKING_MAX_TARGET_PPS        \
    (LINE_TRACKING_MAX_BASE_SPEED_PPS + LINE_TRACKING_MAX_TURN_PPS)
#define LINE_TRACKING_KP                    45.0f
#define LINE_TRACKING_KI                    0.0f
#define LINE_TRACKING_KD                    0.0f
#define LINE_TRACKING_I_LIMIT               1200.0f
#define LINE_TRACKING_DEADBAND              0.3f
#define LINE_TRACKING_LEFT_PWM_LIMIT        1000.0f
#define LINE_TRACKING_RIGHT_PWM_LIMIT       1000.0f
#define LINE_TRACKING_MOTOR_OUTPUT_ENABLED  1
#define LINE_TRACKING_SIGN                  1.0f

static const int16_t g_line_tracking_weights[LINE_SENSOR_I2C_CHANNEL_COUNT] = {
    -35, -25, -15, -5, 5, 15, 25, 35
};

static pid_controller_t g_line_tracking_pid;
static line_tracking_state_t g_line_tracking_state;
static uint32_t g_line_tracking_last_update_ms;
static uint32_t g_line_tracking_enable_ms;
static bool g_line_tracking_initialized;
static bool g_line_tracking_motor_output_enabled =
    (LINE_TRACKING_MOTOR_OUTPUT_ENABLED != 0);
static float g_line_tracking_left_pwm_limit = LINE_TRACKING_LEFT_PWM_LIMIT;
static float g_line_tracking_right_pwm_limit = LINE_TRACKING_RIGHT_PWM_LIMIT;

static float line_tracking_clamp(
    float value, float min_value, float max_value);
static float line_tracking_get_start_scale(uint32_t now_ms);
static int16_t line_tracking_calculate_error(
    const line_sensor_i2c_state_t *sensor, bool *line_seen);
static void line_tracking_apply_speed(float base_speed, float turn_pps);
static void line_tracking_configure_speed_output_limits(void);

void LineTrackingControl_Init(uint32_t now_ms)
{
    PID_Init(&g_line_tracking_pid);
    PID_SetTunings(&g_line_tracking_pid, LINE_TRACKING_KP,
        LINE_TRACKING_KI, LINE_TRACKING_KD);
    PID_SetOutputLimits(&g_line_tracking_pid, -LINE_TRACKING_MAX_TURN_PPS,
        LINE_TRACKING_MAX_TURN_PPS);
    PID_SetIntegralLimits(&g_line_tracking_pid, -LINE_TRACKING_I_LIMIT,
        LINE_TRACKING_I_LIMIT);
    PID_SetDeadband(&g_line_tracking_pid, LINE_TRACKING_DEADBAND);

    LineSensorI2C_Init();
    EncoderSpeedControl_Init(now_ms);

    g_line_tracking_state.raw = 0xFFU;
    g_line_tracking_state.active_mask = 0U;
    g_line_tracking_state.active_count = 0U;
    g_line_tracking_state.sensor_error = 0U;
    g_line_tracking_state.line_error = 0;
    g_line_tracking_state.turn_correction_pps = 0.0f;
    g_line_tracking_state.base_speed_pps = LINE_TRACKING_BASE_SPEED_PPS;
    g_line_tracking_state.left_target_pps = 0.0f;
    g_line_tracking_state.right_target_pps = 0.0f;
    g_line_tracking_state.left_actual_pps = 0;
    g_line_tracking_state.right_actual_pps = 0;
    g_line_tracking_state.sensor_ok = false;
    g_line_tracking_state.line_seen = false;
    g_line_tracking_state.enabled = false;

    g_line_tracking_last_update_ms = now_ms;
    g_line_tracking_enable_ms = now_ms;
    g_line_tracking_initialized = true;
}

void LineTrackingControl_Task(uint32_t now_ms)
{
    line_sensor_i2c_state_t sensor;
    uint32_t elapsed_ms;
    float dt_s;
    float turn_pps;
    float start_scale;
    bool line_seen;

    if (!g_line_tracking_initialized) {
        return;
    }

    EncoderSpeedControl_Task(now_ms);

    elapsed_ms = now_ms - g_line_tracking_last_update_ms;
    if (elapsed_ms < LINE_TRACKING_UPDATE_MS) {
        return;
    }
    if (elapsed_ms > LINE_TRACKING_DT_CLAMP_MS) {
        elapsed_ms = LINE_TRACKING_UPDATE_MS;
    }
    g_line_tracking_last_update_ms = now_ms;
    dt_s = (float) elapsed_ms / 1000.0f;

    g_line_tracking_state.left_actual_pps =
        Encoder_GetSpeedPps(ENCODER_LEFT);
    g_line_tracking_state.right_actual_pps =
        Encoder_GetSpeedPps(ENCODER_RIGHT);

    g_line_tracking_state.sensor_ok = LineSensorI2C_ReadState(&sensor);
    g_line_tracking_state.sensor_error = LineSensorI2C_GetLastError();
    if (!g_line_tracking_state.sensor_ok) {
        g_line_tracking_state.raw = 0xFFU;
        g_line_tracking_state.active_mask = 0U;
        g_line_tracking_state.active_count = 0U;
        g_line_tracking_state.line_error = 0;
        g_line_tracking_state.line_seen = false;
        g_line_tracking_state.turn_correction_pps = 0.0f;
        g_line_tracking_state.left_target_pps = 0.0f;
        g_line_tracking_state.right_target_pps = 0.0f;
        PID_Reset(&g_line_tracking_pid);
        line_tracking_apply_speed(0.0f, 0.0f);
        return;
    }

    g_line_tracking_state.raw = sensor.raw;
    g_line_tracking_state.active_mask = sensor.active_mask;
    g_line_tracking_state.active_count = sensor.active_count;
    g_line_tracking_state.line_error =
        line_tracking_calculate_error(&sensor, &line_seen);
    g_line_tracking_state.line_seen = line_seen;

    if (!g_line_tracking_state.enabled || !line_seen) {
        PID_Reset(&g_line_tracking_pid);
        line_tracking_apply_speed(0.0f, 0.0f);
        return;
    }

    turn_pps = PID_UpdateError(&g_line_tracking_pid,
        (float) g_line_tracking_state.line_error, 0.0f, dt_s);
    turn_pps = line_tracking_clamp(turn_pps, -LINE_TRACKING_MAX_TURN_PPS,
        LINE_TRACKING_MAX_TURN_PPS);

    start_scale = line_tracking_get_start_scale(now_ms);
    turn_pps *= start_scale;
    line_tracking_apply_speed(
        g_line_tracking_state.base_speed_pps * start_scale, turn_pps);
}

void LineTrackingControl_SetEnabled(bool enabled)
{
    if (!g_line_tracking_initialized) {
        return;
    }

    if (!enabled) {
        LineTrackingControl_Stop();
    } else {
        if (!g_line_tracking_state.enabled) {
            PID_Reset(&g_line_tracking_pid);
            line_tracking_apply_speed(0.0f, 0.0f);
            g_line_tracking_enable_ms = g_line_tracking_last_update_ms;
        }
        g_line_tracking_state.enabled = true;
        line_tracking_configure_speed_output_limits();
    }
}

void LineTrackingControl_Start(void)
{
    LineTrackingControl_SetEnabled(true);
}

void LineTrackingControl_Toggle(void)
{
    LineTrackingControl_SetEnabled(!g_line_tracking_state.enabled);
}

bool LineTrackingControl_IsEnabled(void)
{
    return g_line_tracking_state.enabled;
}

void LineTrackingControl_SetBaseSpeedPps(float speed_pps)
{
    g_line_tracking_state.base_speed_pps = line_tracking_clamp(speed_pps,
        0.0f, LINE_TRACKING_MAX_BASE_SPEED_PPS);
}

void LineTrackingControl_SetTunings(float kp, float ki, float kd)
{
    PID_SetTunings(&g_line_tracking_pid, kp, ki, kd);
}

void LineTrackingControl_SetOutputLimits(float output_min, float output_max)
{
    PID_SetOutputLimits(&g_line_tracking_pid, output_min, output_max);
}

void LineTrackingControl_SetIntegralLimits(float integral_min,
    float integral_max)
{
    PID_SetIntegralLimits(&g_line_tracking_pid, integral_min, integral_max);
}

void LineTrackingControl_SetDeadband(float deadband)
{
    PID_SetDeadband(&g_line_tracking_pid, deadband);
}

void LineTrackingControl_SetDriveOutputLimits(float left_pwm_limit,
    float right_pwm_limit)
{
    if (left_pwm_limit < 0.0f) {
        left_pwm_limit = -left_pwm_limit;
    }
    if (right_pwm_limit < 0.0f) {
        right_pwm_limit = -right_pwm_limit;
    }

    g_line_tracking_left_pwm_limit = left_pwm_limit;
    g_line_tracking_right_pwm_limit = right_pwm_limit;

    if (g_line_tracking_initialized) {
        line_tracking_configure_speed_output_limits();
    }
}

void LineTrackingControl_SetMotorOutputEnabled(bool enabled)
{
    g_line_tracking_motor_output_enabled = enabled;
}

void LineTrackingControl_GetTunings(line_tracking_pid_t *pid)
{
    if (pid != NULL) {
        pid->kp = g_line_tracking_pid.kp;
        pid->ki = g_line_tracking_pid.ki;
        pid->kd = g_line_tracking_pid.kd;
    }
}

void LineTrackingControl_GetConfig(line_tracking_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->kp = g_line_tracking_pid.kp;
    config->ki = g_line_tracking_pid.ki;
    config->kd = g_line_tracking_pid.kd;
    config->output_min = g_line_tracking_pid.output_min;
    config->output_max = g_line_tracking_pid.output_max;
    config->integral_min = g_line_tracking_pid.integral_min;
    config->integral_max = g_line_tracking_pid.integral_max;
    config->deadband = g_line_tracking_pid.deadband;
    config->base_speed_pps = g_line_tracking_state.base_speed_pps;
    config->left_pwm_limit = g_line_tracking_left_pwm_limit;
    config->right_pwm_limit = g_line_tracking_right_pwm_limit;
}

void LineTrackingControl_GetState(line_tracking_state_t *state)
{
    if (state != NULL) {
        *state = g_line_tracking_state;
    }
}

void LineTrackingControl_Stop(void)
{
    g_line_tracking_state.enabled = false;
    PID_Reset(&g_line_tracking_pid);
    line_tracking_apply_speed(0.0f, 0.0f);
    EncoderSpeedControl_Stop();
    EncoderSpeedControl_RestoreDefaultOutputLimits();
}

static float line_tracking_clamp(
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

static float line_tracking_get_start_scale(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!g_line_tracking_state.enabled) {
        return 0.0f;
    }

    elapsed_ms = now_ms - g_line_tracking_enable_ms;
    if (elapsed_ms >= LINE_TRACKING_START_RAMP_MS) {
        return 1.0f;
    }

    return (float) elapsed_ms / (float) LINE_TRACKING_START_RAMP_MS;
}

static int16_t line_tracking_calculate_error(
    const line_sensor_i2c_state_t *sensor, bool *line_seen)
{
    int32_t weighted_sum = 0;
    uint8_t i;

    if (line_seen != NULL) {
        *line_seen = false;
    }
    if ((sensor == NULL) || (sensor->active_count == 0U)) {
        return g_line_tracking_state.line_error;
    }

    if (line_seen != NULL) {
        *line_seen = true;
    }

    for (i = 0U; i < LINE_SENSOR_I2C_CHANNEL_COUNT; i++) {
        if (sensor->channel[i]) {
            weighted_sum += g_line_tracking_weights[i];
        }
    }

    return (int16_t) (weighted_sum / (int32_t) sensor->active_count);
}

static void line_tracking_apply_speed(float base_speed, float turn_pps)
{
    float signed_turn = turn_pps * LINE_TRACKING_SIGN;
    float left_target = base_speed + signed_turn;
    float right_target = base_speed - signed_turn;

    if (base_speed <= 0.0f) {
        left_target = 0.0f;
        right_target = 0.0f;
    } else {
        left_target = line_tracking_clamp(left_target,
            -LINE_TRACKING_MAX_TARGET_PPS, LINE_TRACKING_MAX_TARGET_PPS);
        right_target = line_tracking_clamp(right_target,
            -LINE_TRACKING_MAX_TARGET_PPS, LINE_TRACKING_MAX_TARGET_PPS);
    }

    g_line_tracking_state.turn_correction_pps = turn_pps;
    g_line_tracking_state.left_target_pps = left_target;
    g_line_tracking_state.right_target_pps = right_target;

    if (g_line_tracking_motor_output_enabled) {
        EncoderSpeedControl_SetTargetPps(left_target, right_target);
    }
}

static void line_tracking_configure_speed_output_limits(void)
{
    EncoderSpeedControl_SetOutputLimits(g_line_tracking_left_pwm_limit,
        g_line_tracking_right_pwm_limit);
}
