#include "wheel_yaw_control.h"

#include "icm20948.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define WHEEL_YAW_DEFAULT_KP                   45.0f
#define WHEEL_YAW_DEFAULT_KI                    0.8f
#define WHEEL_YAW_DEFAULT_KD                    7.0f
#define WHEEL_YAW_DEFAULT_MAX_TURN_PPS        800.0f
#define WHEEL_YAW_DEFAULT_MIN_TURN_PPS        200.0f
#define WHEEL_YAW_DEFAULT_TOLERANCE_DEG         0.2f
#define WHEEL_YAW_DEFAULT_SETTLE_RATE_DPS       5.0f
#define WHEEL_YAW_DEFAULT_SETTLE_TIME_MS      300U
#define WHEEL_YAW_DEFAULT_FEEDFORWARD_BOOST    40U
#define WHEEL_YAW_KP_MAX                       50.0f
#define WHEEL_YAW_KI_MAX                       20.0f
#define WHEEL_YAW_KD_MAX                       20.0f
#define WHEEL_YAW_MAX_TURN_SPEED_MIN_PPS      100.0f
#define WHEEL_YAW_TOLERANCE_MIN_DEG             0.1f
#define WHEEL_YAW_TOLERANCE_MAX_DEG            15.0f
#define WHEEL_YAW_SETTLE_RATE_MIN_DPS            0.1f
#define WHEEL_YAW_SETTLE_RATE_MAX_DPS           50.0f
#define WHEEL_YAW_SETTLE_TIME_MIN_MS            50U
#define WHEEL_YAW_SETTLE_TIME_MAX_MS          2000U
#define WHEEL_YAW_SETTLE_WHEEL_SPEED_PPS       120
#define WHEEL_YAW_IMU_MAX_AGE_MS                50U
#define WHEEL_YAW_DT_MAX_MS                     30U
#define WHEEL_YAW_INTEGRAL_LIMIT_DEG_S         100.0f
#define WHEEL_YAW_RATE_FILTER_TAU_S               0.05f
#define WHEEL_YAW_TOLERANCE_HYSTERESIS_DEG        0.1f
#define WHEEL_YAW_FEEDFORWARD_RAMP_MULTIPLIER      2.0f

static wheel_yaw_control_config_t g_config;
static wheel_yaw_control_snapshot_t g_snapshot;
static uint32_t g_started_ms;
static uint32_t g_last_update_ms;
static uint32_t g_deadline_ms;
static uint32_t g_settle_started_ms;
static bool g_settle_pending;
static bool g_error_deadband_latched;
static float g_integral_deg_s;
static float g_filtered_yaw_rate_dps;

static wheel_yaw_control_result_t wheel_yaw_start(
    float target_yaw_deg, uint32_t timeout_ms, uint32_t now_ms);
static void wheel_yaw_complete(uint32_t now_ms);
static void wheel_yaw_fault(wheel_yaw_control_result_t result,
    car_control_block_reason_t reason);
static float wheel_yaw_wrap_deg(float angle_deg);
static float wheel_yaw_abs(float value);
static float wheel_yaw_clamp(float value, float minimum, float maximum);
static bool wheel_yaw_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void WheelYawControl_Init(uint32_t now_ms)
{
    g_config.kp = WHEEL_YAW_DEFAULT_KP;
    g_config.ki = WHEEL_YAW_DEFAULT_KI;
    g_config.kd = WHEEL_YAW_DEFAULT_KD;
    g_config.max_turn_speed_pps = WHEEL_YAW_DEFAULT_MAX_TURN_PPS;
    g_config.min_turn_speed_pps = WHEEL_YAW_DEFAULT_MIN_TURN_PPS;
    g_config.tolerance_deg = WHEEL_YAW_DEFAULT_TOLERANCE_DEG;
    g_config.settle_yaw_rate_dps = WHEEL_YAW_DEFAULT_SETTLE_RATE_DPS;
    g_config.settle_time_ms = WHEEL_YAW_DEFAULT_SETTLE_TIME_MS;
    g_config.feedforward_boost_permille =
        WHEEL_YAW_DEFAULT_FEEDFORWARD_BOOST;

    g_snapshot.target_yaw_deg = 0.0f;
    g_snapshot.current_yaw_deg = 0.0f;
    g_snapshot.error_deg = 0.0f;
    g_snapshot.yaw_rate_dps = 0.0f;
    g_snapshot.turn_speed_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_OK;
    g_snapshot.imu_ready = false;
    g_snapshot.running = false;
    g_snapshot.settled = false;

    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_deadline_ms = now_ms;
    g_settle_started_ms = now_ms;
    g_settle_pending = false;
    g_error_deadband_latched = false;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

bool WheelYawControl_ConfigIsValid(
    const wheel_yaw_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    return (config->kp > 0.0f) &&
        (config->kp <= WHEEL_YAW_KP_MAX) &&
        (config->ki >= 0.0f) &&
        (config->ki <= WHEEL_YAW_KI_MAX) &&
        (config->kd >= 0.0f) &&
        (config->kd <= WHEEL_YAW_KD_MAX) &&
        (config->max_turn_speed_pps >=
            WHEEL_YAW_MAX_TURN_SPEED_MIN_PPS) &&
        (config->max_turn_speed_pps <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (config->min_turn_speed_pps >= 0.0f) &&
        (config->min_turn_speed_pps <= config->max_turn_speed_pps) &&
        (config->tolerance_deg >= WHEEL_YAW_TOLERANCE_MIN_DEG) &&
        (config->tolerance_deg <= WHEEL_YAW_TOLERANCE_MAX_DEG) &&
        (config->settle_yaw_rate_dps >=
            WHEEL_YAW_SETTLE_RATE_MIN_DPS) &&
        (config->settle_yaw_rate_dps <=
            WHEEL_YAW_SETTLE_RATE_MAX_DPS) &&
        (config->settle_time_ms >= WHEEL_YAW_SETTLE_TIME_MIN_MS) &&
        (config->settle_time_ms <= WHEEL_YAW_SETTLE_TIME_MAX_MS) &&
        (config->feedforward_boost_permille <=
            WHEEL_SPEED_CONTROL_FEEDFORWARD_BOOST_MAX);
}

wheel_yaw_control_result_t WheelYawControl_SetConfig(
    const wheel_yaw_control_config_t *config)
{
    if (!WheelYawControl_ConfigIsValid(config)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BAD_CONFIG;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    g_config = *config;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_OK;
    return WHEEL_YAW_CONTROL_OK;
}

bool WheelYawControl_GetConfig(wheel_yaw_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

wheel_yaw_control_result_t WheelYawControl_StartAbsolute(
    float target_yaw_deg, uint32_t timeout_ms, uint32_t now_ms)
{
    if (!(target_yaw_deg >= -WHEEL_YAW_CONTROL_TARGET_MAX_DEG) ||
        !(target_yaw_deg <= WHEEL_YAW_CONTROL_TARGET_MAX_DEG)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }
    return wheel_yaw_start(target_yaw_deg, timeout_ms, now_ms);
}

wheel_yaw_control_result_t WheelYawControl_StartRelative(
    float delta_yaw_deg, uint32_t timeout_ms, uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (!((delta_yaw_deg < 0.0f) || (delta_yaw_deg > 0.0f)) ||
        !(delta_yaw_deg >= -WHEEL_YAW_CONTROL_TARGET_MAX_DEG) ||
        !(delta_yaw_deg <= WHEEL_YAW_CONTROL_TARGET_MAX_DEG)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_YAW_IMU_MAX_AGE_MS)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_IMU_ERROR;
        return g_snapshot.last_result;
    }
    return wheel_yaw_start(
        wheel_yaw_wrap_deg(imu.yaw_deg + delta_yaw_deg),
        timeout_ms, now_ms);
}

void WheelYawControl_Task(uint32_t now_ms)
{
    icm20948_snapshot_t imu;
    wheel_speed_control_snapshot_t speed;
    uint32_t elapsed_ms;
    float dt_s;
    float output_pps;
    float abs_error;
    float control_error;
    float yaw_rate_filter_alpha;
    bool within_error;
    bool within_rate;
    bool wheels_stopped;

    if (!g_snapshot.running) {
        return;
    }
    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    if (wheel_yaw_deadline_reached(now_ms, g_deadline_ms)) {
        wheel_yaw_fault(WHEEL_YAW_CONTROL_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (!WheelSpeedControl_GetSnapshot(&speed) || !speed.running ||
        (speed.owner_mode != CAR_CONTROL_MODE_YAW) ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_YAW)) {
        wheel_yaw_fault(WHEEL_YAW_CONTROL_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < WHEEL_YAW_CONTROL_UPDATE_INTERVAL_MS) {
        return;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_YAW_IMU_MAX_AGE_MS)) {
        wheel_yaw_fault(WHEEL_YAW_CONTROL_IMU_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    g_last_update_ms = now_ms;
    g_snapshot.last_interval_ms = elapsed_ms;
    if (elapsed_ms > g_snapshot.max_interval_ms) {
        g_snapshot.max_interval_ms = elapsed_ms;
    }
    if (elapsed_ms > WHEEL_YAW_DT_MAX_MS) {
        elapsed_ms = WHEEL_YAW_CONTROL_UPDATE_INTERVAL_MS;
    }
    dt_s = (float) elapsed_ms / 1000.0f;
    g_snapshot.current_yaw_deg = wheel_yaw_wrap_deg(imu.yaw_deg);
    g_snapshot.yaw_rate_dps = imu.yaw_rate_dps;
    yaw_rate_filter_alpha = dt_s / (WHEEL_YAW_RATE_FILTER_TAU_S + dt_s);
    g_filtered_yaw_rate_dps += yaw_rate_filter_alpha *
        (g_snapshot.yaw_rate_dps - g_filtered_yaw_rate_dps);
    g_snapshot.error_deg = wheel_yaw_wrap_deg(
        g_snapshot.target_yaw_deg - g_snapshot.current_yaw_deg);
    g_snapshot.imu_ready = true;
    abs_error = wheel_yaw_abs(g_snapshot.error_deg);
    control_error = g_snapshot.error_deg;
    if (control_error > 0.0f) {
        control_error -= 0.5f * g_config.tolerance_deg;
    } else {
        control_error += 0.5f * g_config.tolerance_deg;
    }
    if (abs_error <= g_config.tolerance_deg) {
        g_error_deadband_latched = true;
    } else if (abs_error > (g_config.tolerance_deg +
            WHEEL_YAW_TOLERANCE_HYSTERESIS_DEG)) {
        g_error_deadband_latched = false;
    }
    within_error = g_error_deadband_latched;
    within_rate = wheel_yaw_abs(g_snapshot.yaw_rate_dps) <=
        g_config.settle_yaw_rate_dps;
    wheels_stopped =
        (speed.left_measured_pps <= WHEEL_YAW_SETTLE_WHEEL_SPEED_PPS) &&
        (speed.left_measured_pps >= -WHEEL_YAW_SETTLE_WHEEL_SPEED_PPS) &&
        (speed.right_measured_pps <= WHEEL_YAW_SETTLE_WHEEL_SPEED_PPS) &&
        (speed.right_measured_pps >= -WHEEL_YAW_SETTLE_WHEEL_SPEED_PPS);

    if (within_error) {
        output_pps = 0.0f;
        if (within_rate && wheels_stopped) {
            if (!g_settle_pending) {
                g_settle_pending = true;
                g_settle_started_ms = now_ms;
            } else if ((uint32_t) (now_ms - g_settle_started_ms) >=
                g_config.settle_time_ms) {
                wheel_yaw_complete(now_ms);
                return;
            }
        } else {
            g_settle_pending = false;
        }
    } else {
        g_settle_pending = false;
        if ((g_integral_deg_s * control_error) < 0.0f) {
            g_integral_deg_s = 0.0f;
        }
        g_integral_deg_s = wheel_yaw_clamp(
            g_integral_deg_s + control_error * dt_s,
            -WHEEL_YAW_INTEGRAL_LIMIT_DEG_S,
            WHEEL_YAW_INTEGRAL_LIMIT_DEG_S);
        output_pps = g_config.kp * control_error +
            g_config.ki * g_integral_deg_s -
            g_config.kd * g_filtered_yaw_rate_dps;
        output_pps = wheel_yaw_clamp(output_pps,
            -g_config.max_turn_speed_pps,
            g_config.max_turn_speed_pps);
        if (((g_snapshot.error_deg > 0.0f) &&
                (output_pps < 0.0f)) ||
            ((g_snapshot.error_deg < 0.0f) &&
                (output_pps > 0.0f))) {
            output_pps = 0.0f;
        }
        if (wheel_yaw_abs(output_pps) <
            g_config.min_turn_speed_pps && within_rate) {
            output_pps = (g_snapshot.error_deg > 0.0f) ?
                g_config.min_turn_speed_pps :
                -g_config.min_turn_speed_pps;
        }
    }

    g_snapshot.turn_speed_target_pps = output_pps;
    g_snapshot.left_speed_target_pps = -output_pps;
    g_snapshot.right_speed_target_pps = output_pps;
    if (WheelSpeedControl_SetTargets(
            g_snapshot.left_speed_target_pps,
            g_snapshot.right_speed_target_pps,
            now_ms) != WHEEL_SPEED_CONTROL_OK) {
        wheel_yaw_fault(WHEEL_YAW_CONTROL_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_OK;
}

void WheelYawControl_Stop(car_control_block_reason_t reason)
{
    if (g_snapshot.running) {
        WheelSpeedControl_Stop(reason);
    }
    g_snapshot.running = false;
    g_snapshot.settled = false;
    g_snapshot.turn_speed_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_STOPPED;
    g_settle_pending = false;
    g_error_deadband_latched = false;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

bool WheelYawControl_GetSnapshot(wheel_yaw_control_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static wheel_yaw_control_result_t wheel_yaw_start(
    float target_yaw_deg, uint32_t timeout_ms, uint32_t now_ms)
{
    icm20948_snapshot_t imu;
    float feedforward_ramp_pps;

    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    if ((timeout_ms < WHEEL_YAW_CONTROL_RUN_MIN_MS) ||
        (timeout_ms > WHEEL_YAW_CONTROL_RUN_MAX_MS)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_BAD_TIMEOUT;
        return g_snapshot.last_result;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_YAW_IMU_MAX_AGE_MS)) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_IMU_ERROR;
        return g_snapshot.last_result;
    }
    feedforward_ramp_pps = g_config.min_turn_speed_pps *
        WHEEL_YAW_FEEDFORWARD_RAMP_MULTIPLIER;
    if (feedforward_ramp_pps > WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) {
        feedforward_ramp_pps = WHEEL_SPEED_CONTROL_TARGET_MAX_PPS;
    }
    if (WheelSpeedControl_StartForModeWithFeedforward(
            CAR_CONTROL_MODE_YAW,
            g_config.feedforward_boost_permille,
            feedforward_ramp_pps,
            now_ms) != WHEEL_SPEED_CONTROL_OK) {
        g_snapshot.last_result = WHEEL_YAW_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }
    if (WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        g_snapshot.last_result = WHEEL_YAW_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }

    g_snapshot.target_yaw_deg = wheel_yaw_wrap_deg(target_yaw_deg);
    g_snapshot.current_yaw_deg = wheel_yaw_wrap_deg(imu.yaw_deg);
    g_snapshot.error_deg = wheel_yaw_wrap_deg(
        g_snapshot.target_yaw_deg - g_snapshot.current_yaw_deg);
    g_snapshot.yaw_rate_dps = imu.yaw_rate_dps;
    g_snapshot.turn_speed_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_OK;
    g_snapshot.imu_ready = true;
    g_snapshot.running = true;
    g_snapshot.settled = false;
    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_deadline_ms = now_ms + timeout_ms;
    g_settle_started_ms = now_ms;
    g_settle_pending = false;
    g_error_deadband_latched = false;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = imu.yaw_rate_dps;
    return WHEEL_YAW_CONTROL_OK;
}

static void wheel_yaw_complete(uint32_t now_ms)
{
    (void) WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms);
    WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_TEST_COMPLETE);
    g_snapshot.running = false;
    g_snapshot.settled = true;
    g_snapshot.turn_speed_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_YAW_CONTROL_OK;
    g_settle_pending = false;
    g_error_deadband_latched = false;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

static void wheel_yaw_fault(wheel_yaw_control_result_t result,
    car_control_block_reason_t reason)
{
    WheelSpeedControl_Stop(reason);
    g_snapshot.running = false;
    g_snapshot.settled = false;
    g_snapshot.turn_speed_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = result;
    g_settle_pending = false;
    g_error_deadband_latched = false;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

static float wheel_yaw_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float wheel_yaw_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float wheel_yaw_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool wheel_yaw_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
