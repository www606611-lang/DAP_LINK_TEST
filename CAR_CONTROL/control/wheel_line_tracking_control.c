#include "wheel_line_tracking_control.h"

#include "wheel_speed_control.h"

#include <stddef.h>

#define WHEEL_LINE_TRACKING_DEFAULT_KP                  12.0f
#define WHEEL_LINE_TRACKING_DEFAULT_KI                   0.0f
#define WHEEL_LINE_TRACKING_DEFAULT_KD                   0.0f
#define WHEEL_LINE_TRACKING_DEFAULT_MAX_CORRECTION_PPS 400.0f
#define WHEEL_LINE_TRACKING_DEFAULT_DEADBAND              2.0f
#define WHEEL_LINE_TRACKING_KP_MAX                      100.0f
#define WHEEL_LINE_TRACKING_KI_MAX                      100.0f
#define WHEEL_LINE_TRACKING_KD_MAX                       20.0f
#define WHEEL_LINE_TRACKING_DEADBAND_MAX                 20.0f
#define WHEEL_LINE_TRACKING_ERROR_MAX                    35
#define WHEEL_LINE_TRACKING_BASE_SPEED_MIN_PPS          100.0f
#define WHEEL_LINE_TRACKING_DT_MAX_MS                    60U
#define WHEEL_LINE_TRACKING_INTEGRAL_LIMIT              100.0f
#define WHEEL_LINE_TRACKING_DERIVATIVE_TAU_S              0.04f
#define WHEEL_LINE_TRACKING_WIDE_LINE_COUNT                4U
#define WHEEL_LINE_TRACKING_CORNER_BASE_SPEED_PPS        250.0f
#define WHEEL_LINE_TRACKING_CORNER_EXIT_STABLE_MS         600U
#define WHEEL_LINE_TRACKING_CORNER_MIN_CORRECTION_PPS    350.0f
#define WHEEL_LINE_TRACKING_CORNER_SEARCH_CORRECTION_PPS 500.0f
#define WHEEL_LINE_TRACKING_CORNER_REACQUIRE_MS          1200U
#define WHEEL_LINE_TRACKING_BASE_ACCEL_PPS_PER_S         600.0f
#define WHEEL_LINE_TRACKING_SLOWDOWN_FRACTION               0.65f

static wheel_line_tracking_config_t g_config;
static wheel_line_tracking_snapshot_t g_snapshot;
static uint32_t g_started_ms;
static uint32_t g_last_update_ms;
static uint32_t g_last_command_ms;
static uint32_t g_observation_ms;
static uint32_t g_corner_centered_since_ms;
static uint32_t g_corner_line_lost_since_ms;
static float g_requested_base_speed_pps;
static float g_effective_base_speed_pps;
static float g_integral;
static float g_previous_error;
static float g_filtered_derivative;
static int8_t g_corner_turn_sign;
static int8_t g_last_turn_sign;
static bool g_corner_active;
static bool g_corner_recovery_active;
static bool g_corner_line_search_active;

static bool wheel_line_tracking_speed_is_valid(float speed_pps);
static bool wheel_line_tracking_observation_is_valid(
    int16_t line_error, bool line_seen, uint32_t observation_ms,
    uint32_t now_ms);
static void wheel_line_tracking_fault(
    wheel_line_tracking_result_t result, car_control_block_reason_t reason);
static float wheel_line_tracking_abs(float value);
static float wheel_line_tracking_clamp(
    float value, float minimum, float maximum);
static float wheel_line_tracking_effective_base_speed(
    float requested_base_speed_pps, float correction_pps);
static bool wheel_line_tracking_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void WheelLineTrackingControl_Init(uint32_t now_ms)
{
    g_config.kp = WHEEL_LINE_TRACKING_DEFAULT_KP;
    g_config.ki = WHEEL_LINE_TRACKING_DEFAULT_KI;
    g_config.kd = WHEEL_LINE_TRACKING_DEFAULT_KD;
    g_config.max_correction_pps =
        WHEEL_LINE_TRACKING_DEFAULT_MAX_CORRECTION_PPS;
    g_config.deadband = WHEEL_LINE_TRACKING_DEFAULT_DEADBAND;

    g_snapshot.line_error = 0;
    g_snapshot.active_count = 0U;
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.observation_age_ms = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    g_snapshot.line_seen = false;
    g_snapshot.running = false;

    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_observation_ms = now_ms;
    g_corner_centered_since_ms = 0U;
    g_corner_line_lost_since_ms = now_ms;
    g_requested_base_speed_pps = 0.0f;
    g_effective_base_speed_pps = 0.0f;
    g_integral = 0.0f;
    g_previous_error = 0.0f;
    g_filtered_derivative = 0.0f;
    g_corner_turn_sign = 0;
    g_last_turn_sign = 0;
    g_corner_active = false;
    g_corner_recovery_active = false;
    g_corner_line_search_active = false;
}

bool WheelLineTrackingControl_ConfigIsValid(
    const wheel_line_tracking_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    return (config->kp > 0.0f) &&
        (config->kp <= WHEEL_LINE_TRACKING_KP_MAX) &&
        (config->ki >= 0.0f) &&
        (config->ki <= WHEEL_LINE_TRACKING_KI_MAX) &&
        (config->kd >= 0.0f) &&
        (config->kd <= WHEEL_LINE_TRACKING_KD_MAX) &&
        (config->max_correction_pps > 0.0f) &&
        (config->max_correction_pps <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (config->deadband >= 0.0f) &&
        (config->deadband <= WHEEL_LINE_TRACKING_DEADBAND_MAX);
}

wheel_line_tracking_result_t WheelLineTrackingControl_SetConfig(
    const wheel_line_tracking_config_t *config)
{
    if (!WheelLineTrackingControl_ConfigIsValid(config)) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_BAD_CONFIG;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_BUSY;
        return g_snapshot.last_result;
    }
    g_config = *config;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    return WHEEL_LINE_TRACKING_OK;
}

bool WheelLineTrackingControl_GetConfig(
    wheel_line_tracking_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

wheel_line_tracking_result_t WheelLineTrackingControl_Start(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen,
    uint32_t observation_ms, uint32_t now_ms)
{
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_BUSY;
        return g_snapshot.last_result;
    }
    if (!wheel_line_tracking_speed_is_valid(base_speed_pps)) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_BAD_COMMAND;
        return g_snapshot.last_result;
    }
    if ((active_count == 0U) || (active_count > 8U) ||
        !wheel_line_tracking_observation_is_valid(
            line_error, line_seen, observation_ms, now_ms)) {
        g_snapshot.last_result = line_seen ?
            WHEEL_LINE_TRACKING_SENSOR_STALE :
            WHEEL_LINE_TRACKING_LINE_LOST;
        return g_snapshot.last_result;
    }
    if (WheelSpeedControl_StartForMode(
            CAR_CONTROL_MODE_LINE_TRACKING, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_SPEED_ERROR;
        return g_snapshot.last_result;
    }
    if (WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        g_snapshot.last_result = WHEEL_LINE_TRACKING_SPEED_ERROR;
        return g_snapshot.last_result;
    }

    g_snapshot.line_error = line_error;
    g_snapshot.active_count = active_count;
    g_snapshot.base_speed_target_pps = base_speed_pps;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.observation_age_ms = now_ms - observation_ms;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    g_snapshot.line_seen = true;
    g_snapshot.running = true;
    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_observation_ms = observation_ms;
    g_corner_centered_since_ms = 0U;
    g_corner_line_lost_since_ms = now_ms;
    g_requested_base_speed_pps = base_speed_pps;
    g_integral = 0.0f;
    g_previous_error = -(float) line_error;
    g_filtered_derivative = 0.0f;
    g_last_turn_sign = ((float) line_error < -g_config.deadband) ?
        1 : (((float) line_error > g_config.deadband) ? -1 : 0);
    g_corner_active =
        active_count >= WHEEL_LINE_TRACKING_WIDE_LINE_COUNT;
    g_corner_turn_sign = g_corner_active ? g_last_turn_sign : 0;
    g_corner_recovery_active = false;
    g_corner_line_search_active = false;
    g_effective_base_speed_pps =
        (g_corner_active &&
            (base_speed_pps > WHEEL_LINE_TRACKING_CORNER_BASE_SPEED_PPS)) ?
        WHEEL_LINE_TRACKING_CORNER_BASE_SPEED_PPS : base_speed_pps;
    return WHEEL_LINE_TRACKING_OK;
}

wheel_line_tracking_result_t WheelLineTrackingControl_SetCommand(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen,
    uint32_t observation_ms, uint32_t now_ms)
{
    if (!g_snapshot.running) {
        g_snapshot.last_result = WHEEL_LINE_TRACKING_NOT_RUNNING;
        return g_snapshot.last_result;
    }
    if (!wheel_line_tracking_speed_is_valid(base_speed_pps) ||
        (line_error < -WHEEL_LINE_TRACKING_ERROR_MAX) ||
        (line_error > WHEEL_LINE_TRACKING_ERROR_MAX)) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_BAD_COMMAND,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_LINE_TRACKING_BAD_COMMAND;
    }
    if ((uint32_t) (now_ms - observation_ms) >
        WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_SENSOR_STALE,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_LINE_TRACKING_SENSOR_STALE;
    }
    if (!line_seen) {
        if (!g_corner_active || (g_corner_turn_sign == 0)) {
            wheel_line_tracking_fault(WHEEL_LINE_TRACKING_LINE_LOST,
                CAR_CONTROL_BLOCK_EMERGENCY_STOP);
            return WHEEL_LINE_TRACKING_LINE_LOST;
        }
        if (!g_corner_line_search_active) {
            g_corner_line_lost_since_ms = now_ms;
            g_corner_line_search_active = true;
        } else if (wheel_line_tracking_deadline_reached(
                now_ms, g_corner_line_lost_since_ms +
                    WHEEL_LINE_TRACKING_CORNER_REACQUIRE_MS)) {
            wheel_line_tracking_fault(WHEEL_LINE_TRACKING_LINE_LOST,
                CAR_CONTROL_BLOCK_EMERGENCY_STOP);
            return WHEEL_LINE_TRACKING_LINE_LOST;
        }
        g_snapshot.active_count = 0U;
        g_snapshot.command_age_ms = 0U;
        g_snapshot.observation_age_ms = now_ms - observation_ms;
        g_snapshot.line_seen = false;
        g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
        g_last_command_ms = now_ms;
        g_observation_ms = observation_ms;
        g_requested_base_speed_pps = base_speed_pps;
        return WHEEL_LINE_TRACKING_OK;
    }
    if ((active_count == 0U) || (active_count > 8U)) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_BAD_COMMAND,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_LINE_TRACKING_BAD_COMMAND;
    }

    g_corner_line_search_active = false;

    g_snapshot.line_error = line_error;
    g_snapshot.active_count = active_count;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.observation_age_ms = now_ms - observation_ms;
    g_snapshot.line_seen = true;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    g_last_command_ms = now_ms;
    g_observation_ms = observation_ms;
    if (active_count >= WHEEL_LINE_TRACKING_WIDE_LINE_COUNT) {
        bool entering_corner = !g_corner_active;

        g_corner_active = true;
        g_corner_recovery_active = false;
        g_corner_centered_since_ms = 0U;
        if (entering_corner) {
            if ((g_last_turn_sign == 0) &&
                (wheel_line_tracking_abs((float) line_error) >
                    g_config.deadband)) {
                g_last_turn_sign = (line_error < 0) ? 1 : -1;
            }
            g_corner_turn_sign = g_last_turn_sign;
        }
    } else {
        if (!g_corner_active && !g_corner_recovery_active &&
            (wheel_line_tracking_abs((float) line_error) >
                g_config.deadband)) {
            g_last_turn_sign = (line_error < 0) ? 1 : -1;
        }
        if ((g_corner_active || g_corner_recovery_active) &&
            (active_count <= 3U) && (g_corner_turn_sign == 0) &&
            (wheel_line_tracking_abs((float) line_error) >
                g_config.deadband)) {
            g_corner_turn_sign = (line_error < 0) ? 1 : -1;
            g_corner_centered_since_ms = 0U;
        }
        if (g_corner_active && (active_count <= 2U) &&
            (wheel_line_tracking_abs((float) line_error) <=
                g_config.deadband)) {
            if (g_corner_centered_since_ms == 0U) {
                g_corner_centered_since_ms = now_ms;
            } else if (wheel_line_tracking_deadline_reached(
                    now_ms, g_corner_centered_since_ms +
                        WHEEL_LINE_TRACKING_CORNER_EXIT_STABLE_MS)) {
                g_corner_active = false;
                g_corner_recovery_active = true;
                g_corner_centered_since_ms = 0U;
            }
        } else if (g_corner_active) {
            g_corner_centered_since_ms = 0U;
        }
    }
    g_requested_base_speed_pps = base_speed_pps;
    return WHEEL_LINE_TRACKING_OK;
}

void WheelLineTrackingControl_Task(uint32_t now_ms)
{
    wheel_speed_control_snapshot_t speed;
    uint32_t elapsed_ms;
    float dt_s;
    float control_error;
    float derivative;
    float derivative_alpha;
    float candidate_integral;
    float correction_limit;
    float correction_pps;
    float headroom_pps;
    float desired_base_speed_pps;
    float maximum_base_increase_pps;

    if (!g_snapshot.running) {
        return;
    }
    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    g_snapshot.command_age_ms = now_ms - g_last_command_ms;
    g_snapshot.observation_age_ms = now_ms - g_observation_ms;
    if (wheel_line_tracking_deadline_reached(now_ms,
            g_last_command_ms + WHEEL_LINE_TRACKING_COMMAND_LEASE_MS)) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_COMMAND_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (g_snapshot.observation_age_ms >
        WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_SENSOR_STALE,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!WheelSpeedControl_GetSnapshot(&speed) || !speed.running ||
        (speed.owner_mode != CAR_CONTROL_MODE_LINE_TRACKING) ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_LINE_TRACKING)) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < WHEEL_LINE_TRACKING_UPDATE_INTERVAL_MS) {
        return;
    }
    g_last_update_ms = now_ms;
    g_snapshot.last_interval_ms = elapsed_ms;
    if (elapsed_ms > g_snapshot.max_interval_ms) {
        g_snapshot.max_interval_ms = elapsed_ms;
    }
    if (elapsed_ms > WHEEL_LINE_TRACKING_DT_MAX_MS) {
        elapsed_ms = WHEEL_LINE_TRACKING_UPDATE_INTERVAL_MS;
    }
    dt_s = (float) elapsed_ms / 1000.0f;

    control_error = g_snapshot.line_seen ?
        -(float) g_snapshot.line_error : 0.0f;
    if (wheel_line_tracking_abs(control_error) <= g_config.deadband) {
        control_error = 0.0f;
    } else if (control_error > 0.0f) {
        control_error -= g_config.deadband;
    } else {
        control_error += g_config.deadband;
    }
    if ((g_integral * control_error) < 0.0f) {
        g_integral = 0.0f;
    }
    candidate_integral = wheel_line_tracking_clamp(
        g_integral + control_error * dt_s,
        -WHEEL_LINE_TRACKING_INTEGRAL_LIMIT,
        WHEEL_LINE_TRACKING_INTEGRAL_LIMIT);
    derivative = (control_error - g_previous_error) / dt_s;
    derivative_alpha = dt_s /
        (WHEEL_LINE_TRACKING_DERIVATIVE_TAU_S + dt_s);
    g_filtered_derivative += derivative_alpha *
        (derivative - g_filtered_derivative);

    correction_limit = g_config.max_correction_pps;
    headroom_pps = WHEEL_SPEED_CONTROL_TARGET_MAX_PPS -
        wheel_line_tracking_abs(g_requested_base_speed_pps);
    if (correction_limit > headroom_pps) {
        correction_limit = headroom_pps;
    }
    correction_pps = g_config.kp * control_error +
        g_config.ki * candidate_integral +
        g_config.kd * g_filtered_derivative;
    if ((wheel_line_tracking_abs(correction_pps) <= correction_limit) ||
        ((correction_pps * control_error) < 0.0f)) {
        g_integral = candidate_integral;
    }
    correction_pps = g_config.kp * control_error +
        g_config.ki * g_integral +
        g_config.kd * g_filtered_derivative;
    correction_pps = wheel_line_tracking_clamp(
        correction_pps, -correction_limit, correction_limit);
    if ((g_corner_active || g_corner_recovery_active) &&
        (g_corner_turn_sign != 0) &&
        ((g_snapshot.active_count >= WHEEL_LINE_TRACKING_WIDE_LINE_COUNT) ||
            (control_error != 0.0f) || !g_snapshot.line_seen)) {
        float corner_correction =
            wheel_line_tracking_abs(correction_pps);

        if (corner_correction <
            WHEEL_LINE_TRACKING_CORNER_MIN_CORRECTION_PPS) {
            corner_correction =
                WHEEL_LINE_TRACKING_CORNER_MIN_CORRECTION_PPS;
        }
        if (!g_snapshot.line_seen &&
            (corner_correction <
                WHEEL_LINE_TRACKING_CORNER_SEARCH_CORRECTION_PPS)) {
            corner_correction =
                WHEEL_LINE_TRACKING_CORNER_SEARCH_CORRECTION_PPS;
        }
        if (corner_correction > correction_limit) {
            corner_correction = correction_limit;
        }
        correction_pps = (g_corner_turn_sign > 0) ?
            corner_correction : -corner_correction;
    }
    g_previous_error = control_error;

    desired_base_speed_pps = wheel_line_tracking_effective_base_speed(
        g_requested_base_speed_pps, correction_pps);
    if (desired_base_speed_pps <= g_effective_base_speed_pps) {
        g_effective_base_speed_pps = desired_base_speed_pps;
    } else {
        maximum_base_increase_pps =
            WHEEL_LINE_TRACKING_BASE_ACCEL_PPS_PER_S * dt_s;
        g_effective_base_speed_pps += maximum_base_increase_pps;
        if (g_effective_base_speed_pps > desired_base_speed_pps) {
            g_effective_base_speed_pps = desired_base_speed_pps;
        }
    }
    g_snapshot.base_speed_target_pps = g_effective_base_speed_pps;
    if (g_corner_recovery_active && (control_error == 0.0f) &&
        (g_snapshot.active_count <= 2U) &&
        (g_effective_base_speed_pps >=
            (g_requested_base_speed_pps - 1.0f))) {
        g_corner_recovery_active = false;
        g_corner_turn_sign = 0;
    }
    g_snapshot.correction_target_pps = correction_pps;
    g_snapshot.left_speed_target_pps =
        g_snapshot.base_speed_target_pps - correction_pps;
    g_snapshot.right_speed_target_pps =
        g_snapshot.base_speed_target_pps + correction_pps;
    if (WheelSpeedControl_SetTargets(
            g_snapshot.left_speed_target_pps,
            g_snapshot.right_speed_target_pps,
            now_ms) != WHEEL_SPEED_CONTROL_OK) {
        wheel_line_tracking_fault(WHEEL_LINE_TRACKING_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
}

void WheelLineTrackingControl_Stop(car_control_block_reason_t reason)
{
    if (g_snapshot.running) {
        WheelSpeedControl_Stop(reason);
    }
    g_snapshot.running = false;
    g_snapshot.active_count = 0U;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = (reason == CAR_CONTROL_BLOCK_TEST_COMPLETE) ?
        WHEEL_LINE_TRACKING_OK : WHEEL_LINE_TRACKING_STOPPED;
    g_integral = 0.0f;
    g_previous_error = 0.0f;
    g_filtered_derivative = 0.0f;
    g_corner_centered_since_ms = 0U;
    g_corner_line_lost_since_ms = 0U;
    g_requested_base_speed_pps = 0.0f;
    g_effective_base_speed_pps = 0.0f;
    g_corner_turn_sign = 0;
    g_last_turn_sign = 0;
    g_corner_active = false;
    g_corner_recovery_active = false;
    g_corner_line_search_active = false;
}

bool WheelLineTrackingControl_GetSnapshot(
    wheel_line_tracking_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static bool wheel_line_tracking_speed_is_valid(float speed_pps)
{
    return (speed_pps >= WHEEL_LINE_TRACKING_BASE_SPEED_MIN_PPS) &&
        (speed_pps <= WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        ((speed_pps + g_config.max_correction_pps) <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS);
}

static bool wheel_line_tracking_observation_is_valid(
    int16_t line_error, bool line_seen, uint32_t observation_ms,
    uint32_t now_ms)
{
    return line_seen &&
        (line_error >= -WHEEL_LINE_TRACKING_ERROR_MAX) &&
        (line_error <= WHEEL_LINE_TRACKING_ERROR_MAX) &&
        ((uint32_t) (now_ms - observation_ms) <=
            WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS);
}

static void wheel_line_tracking_fault(
    wheel_line_tracking_result_t result, car_control_block_reason_t reason)
{
    WheelSpeedControl_Stop(reason);
    g_snapshot.running = false;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = result;
    g_integral = 0.0f;
    g_previous_error = 0.0f;
    g_filtered_derivative = 0.0f;
    g_corner_centered_since_ms = 0U;
    g_corner_line_lost_since_ms = 0U;
    g_requested_base_speed_pps = 0.0f;
    g_effective_base_speed_pps = 0.0f;
    g_corner_turn_sign = 0;
    g_last_turn_sign = 0;
    g_corner_active = false;
    g_corner_recovery_active = false;
    g_corner_line_search_active = false;
}

static float wheel_line_tracking_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float wheel_line_tracking_clamp(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float wheel_line_tracking_effective_base_speed(
    float requested_base_speed_pps, float correction_pps)
{
    float correction_ratio = wheel_line_tracking_abs(correction_pps) /
        g_config.max_correction_pps;
    float minimum_base_speed_pps =
        (requested_base_speed_pps <
            WHEEL_LINE_TRACKING_CORNER_BASE_SPEED_PPS) ?
        requested_base_speed_pps :
        WHEEL_LINE_TRACKING_CORNER_BASE_SPEED_PPS;
    float base_speed_pps = requested_base_speed_pps *
        (1.0f - WHEEL_LINE_TRACKING_SLOWDOWN_FRACTION *
            correction_ratio);

    if (g_corner_active && (base_speed_pps > minimum_base_speed_pps)) {
        base_speed_pps = minimum_base_speed_pps;
    }
    if (base_speed_pps < minimum_base_speed_pps) {
        base_speed_pps = minimum_base_speed_pps;
    }
    return base_speed_pps;
}

static bool wheel_line_tracking_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
