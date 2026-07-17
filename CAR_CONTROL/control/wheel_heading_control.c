#include "wheel_heading_control.h"

#include "icm20948.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define WHEEL_HEADING_DEFAULT_KP                  30.0f
#define WHEEL_HEADING_DEFAULT_KI                   3.0f
#define WHEEL_HEADING_DEFAULT_KD                   1.5f
#define WHEEL_HEADING_DEFAULT_MAX_CORRECTION_PPS 400.0f
#define WHEEL_HEADING_DEFAULT_DEADBAND_DEG          0.5f
#define WHEEL_HEADING_KP_MAX                       50.0f
#define WHEEL_HEADING_KI_MAX                       20.0f
#define WHEEL_HEADING_KD_MAX                       20.0f
#define WHEEL_HEADING_CORRECTION_MIN_PPS           10.0f
#define WHEEL_HEADING_DEADBAND_MAX_DEG             15.0f
#define WHEEL_HEADING_IMU_MAX_AGE_MS               50U
#define WHEEL_HEADING_DT_MAX_MS                    30U
#define WHEEL_HEADING_INTEGRAL_LIMIT_DEG_S        100.0f
#define WHEEL_HEADING_RATE_FILTER_TAU_S              0.05f

static wheel_heading_control_config_t g_config;
static wheel_heading_control_snapshot_t g_snapshot;
static uint32_t g_started_ms;
static uint32_t g_last_update_ms;
static uint32_t g_last_command_ms;
static float g_integral_deg_s;
static float g_filtered_yaw_rate_dps;

static wheel_heading_control_result_t wheel_heading_start(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms);
static void wheel_heading_fault(wheel_heading_control_result_t result,
    car_control_block_reason_t reason);
static bool wheel_heading_target_is_valid(float target_yaw_deg);
static bool wheel_heading_speed_is_valid(float speed_pps);
static float wheel_heading_wrap_deg(float angle_deg);
static float wheel_heading_abs(float value);
static float wheel_heading_clamp(
    float value, float minimum, float maximum);
static bool wheel_heading_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void WheelHeadingControl_Init(uint32_t now_ms)
{
    g_config.kp = WHEEL_HEADING_DEFAULT_KP;
    g_config.ki = WHEEL_HEADING_DEFAULT_KI;
    g_config.kd = WHEEL_HEADING_DEFAULT_KD;
    g_config.max_correction_pps =
        WHEEL_HEADING_DEFAULT_MAX_CORRECTION_PPS;
    g_config.deadband_deg = WHEEL_HEADING_DEFAULT_DEADBAND_DEG;

    g_snapshot.target_yaw_deg = 0.0f;
    g_snapshot.current_yaw_deg = 0.0f;
    g_snapshot.error_deg = 0.0f;
    g_snapshot.yaw_rate_dps = 0.0f;
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_HEADING_CONTROL_OK;
    g_snapshot.imu_ready = false;
    g_snapshot.running = false;

    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

bool WheelHeadingControl_ConfigIsValid(
    const wheel_heading_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    return (config->kp > 0.0f) &&
        (config->kp <= WHEEL_HEADING_KP_MAX) &&
        (config->ki >= 0.0f) &&
        (config->ki <= WHEEL_HEADING_KI_MAX) &&
        (config->kd >= 0.0f) &&
        (config->kd <= WHEEL_HEADING_KD_MAX) &&
        (config->max_correction_pps >=
            WHEEL_HEADING_CORRECTION_MIN_PPS) &&
        (config->max_correction_pps <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (config->deadband_deg >= 0.0f) &&
        (config->deadband_deg <= WHEEL_HEADING_DEADBAND_MAX_DEG);
}

wheel_heading_control_result_t WheelHeadingControl_SetConfig(
    const wheel_heading_control_config_t *config)
{
    if (!WheelHeadingControl_ConfigIsValid(config)) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_BAD_CONFIG;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    g_config = *config;
    g_snapshot.last_result = WHEEL_HEADING_CONTROL_OK;
    return WHEEL_HEADING_CONTROL_OK;
}

bool WheelHeadingControl_GetConfig(
    wheel_heading_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

wheel_heading_control_result_t WheelHeadingControl_StartAbsolute(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms)
{
    if (!wheel_heading_target_is_valid(target_yaw_deg)) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }
    return wheel_heading_start(target_yaw_deg, base_speed_pps, now_ms);
}

wheel_heading_control_result_t WheelHeadingControl_StartHoldCurrent(
    float base_speed_pps, uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_HEADING_IMU_MAX_AGE_MS)) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_IMU_ERROR;
        return g_snapshot.last_result;
    }
    return wheel_heading_start(
        wheel_heading_wrap_deg(imu.yaw_deg), base_speed_pps, now_ms);
}

wheel_heading_control_result_t WheelHeadingControl_SetCommand(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms)
{
    if (!g_snapshot.running) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_NOT_RUNNING;
        return g_snapshot.last_result;
    }
    if (!wheel_heading_target_is_valid(target_yaw_deg) ||
        !wheel_heading_speed_is_valid(base_speed_pps)) {
        wheel_heading_fault(WHEEL_HEADING_CONTROL_BAD_TARGET,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_HEADING_CONTROL_BAD_TARGET;
    }
    g_snapshot.target_yaw_deg = wheel_heading_wrap_deg(target_yaw_deg);
    g_snapshot.base_speed_target_pps = base_speed_pps;
    g_snapshot.command_age_ms = 0U;
    g_last_command_ms = now_ms;
    g_snapshot.last_result = WHEEL_HEADING_CONTROL_OK;
    return WHEEL_HEADING_CONTROL_OK;
}

void WheelHeadingControl_Task(uint32_t now_ms)
{
    icm20948_snapshot_t imu;
    wheel_speed_control_snapshot_t speed;
    uint32_t elapsed_ms;
    float dt_s;
    float control_error;
    float candidate_integral;
    float correction_pps;
    float correction_limit;
    float headroom_pps;
    float yaw_rate_filter_alpha;

    if (!g_snapshot.running) {
        return;
    }
    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    g_snapshot.command_age_ms = now_ms - g_last_command_ms;
    if (wheel_heading_deadline_reached(now_ms,
            g_last_command_ms + WHEEL_HEADING_CONTROL_COMMAND_LEASE_MS)) {
        wheel_heading_fault(WHEEL_HEADING_CONTROL_COMMAND_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (!WheelSpeedControl_GetSnapshot(&speed) || !speed.running ||
        (speed.owner_mode != CAR_CONTROL_MODE_HEADING) ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_HEADING)) {
        wheel_heading_fault(WHEEL_HEADING_CONTROL_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < WHEEL_HEADING_CONTROL_UPDATE_INTERVAL_MS) {
        return;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_HEADING_IMU_MAX_AGE_MS)) {
        wheel_heading_fault(WHEEL_HEADING_CONTROL_IMU_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    g_last_update_ms = now_ms;
    g_snapshot.last_interval_ms = elapsed_ms;
    if (elapsed_ms > g_snapshot.max_interval_ms) {
        g_snapshot.max_interval_ms = elapsed_ms;
    }
    if (elapsed_ms > WHEEL_HEADING_DT_MAX_MS) {
        elapsed_ms = WHEEL_HEADING_CONTROL_UPDATE_INTERVAL_MS;
    }
    dt_s = (float) elapsed_ms / 1000.0f;
    g_snapshot.current_yaw_deg = wheel_heading_wrap_deg(imu.yaw_deg);
    g_snapshot.yaw_rate_dps = imu.yaw_rate_dps;
    g_snapshot.error_deg = wheel_heading_wrap_deg(
        g_snapshot.target_yaw_deg - g_snapshot.current_yaw_deg);
    g_snapshot.imu_ready = true;

    yaw_rate_filter_alpha = dt_s /
        (WHEEL_HEADING_RATE_FILTER_TAU_S + dt_s);
    g_filtered_yaw_rate_dps += yaw_rate_filter_alpha *
        (g_snapshot.yaw_rate_dps - g_filtered_yaw_rate_dps);
    control_error = g_snapshot.error_deg;
    if (wheel_heading_abs(control_error) <= g_config.deadband_deg) {
        control_error = 0.0f;
    } else if (control_error > 0.0f) {
        control_error -= g_config.deadband_deg;
    } else {
        control_error += g_config.deadband_deg;
    }
    if ((g_integral_deg_s * control_error) < 0.0f) {
        g_integral_deg_s = 0.0f;
    }
    candidate_integral = wheel_heading_clamp(
        g_integral_deg_s + control_error * dt_s,
        -WHEEL_HEADING_INTEGRAL_LIMIT_DEG_S,
        WHEEL_HEADING_INTEGRAL_LIMIT_DEG_S);
    correction_limit = g_config.max_correction_pps;
    headroom_pps = WHEEL_SPEED_CONTROL_TARGET_MAX_PPS -
        wheel_heading_abs(g_snapshot.base_speed_target_pps);
    if (correction_limit > headroom_pps) {
        correction_limit = headroom_pps;
    }
    correction_pps = g_config.kp * control_error +
        g_config.ki * candidate_integral -
        g_config.kd * g_filtered_yaw_rate_dps;
    if ((wheel_heading_abs(correction_pps) <= correction_limit) ||
        ((correction_pps * control_error) < 0.0f)) {
        g_integral_deg_s = candidate_integral;
    }
    correction_pps = g_config.kp * control_error +
        g_config.ki * g_integral_deg_s -
        g_config.kd * g_filtered_yaw_rate_dps;
    correction_pps = wheel_heading_clamp(
        correction_pps, -correction_limit, correction_limit);

    g_snapshot.correction_target_pps = correction_pps;
    g_snapshot.left_speed_target_pps =
        g_snapshot.base_speed_target_pps - correction_pps;
    g_snapshot.right_speed_target_pps =
        g_snapshot.base_speed_target_pps + correction_pps;
    if (WheelSpeedControl_SetTargets(
            g_snapshot.left_speed_target_pps,
            g_snapshot.right_speed_target_pps,
            now_ms) != WHEEL_SPEED_CONTROL_OK) {
        wheel_heading_fault(WHEEL_HEADING_CONTROL_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_HEADING_CONTROL_OK;
}

void WheelHeadingControl_Stop(car_control_block_reason_t reason)
{
    if (g_snapshot.running) {
        WheelSpeedControl_Stop(reason);
    }
    g_snapshot.running = false;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = (reason == CAR_CONTROL_BLOCK_TEST_COMPLETE) ?
        WHEEL_HEADING_CONTROL_OK : WHEEL_HEADING_CONTROL_STOPPED;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

bool WheelHeadingControl_GetSnapshot(
    wheel_heading_control_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static wheel_heading_control_result_t wheel_heading_start(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    if (!wheel_heading_speed_is_valid(base_speed_pps)) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            WHEEL_HEADING_IMU_MAX_AGE_MS)) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_IMU_ERROR;
        return g_snapshot.last_result;
    }
    if (WheelSpeedControl_StartForMode(
            CAR_CONTROL_MODE_HEADING, now_ms) != WHEEL_SPEED_CONTROL_OK) {
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }
    if (WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        g_snapshot.last_result = WHEEL_HEADING_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }

    g_snapshot.target_yaw_deg = wheel_heading_wrap_deg(target_yaw_deg);
    g_snapshot.current_yaw_deg = wheel_heading_wrap_deg(imu.yaw_deg);
    g_snapshot.error_deg = wheel_heading_wrap_deg(
        g_snapshot.target_yaw_deg - g_snapshot.current_yaw_deg);
    g_snapshot.yaw_rate_dps = imu.yaw_rate_dps;
    g_snapshot.base_speed_target_pps = base_speed_pps;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = WHEEL_HEADING_CONTROL_OK;
    g_snapshot.imu_ready = true;
    g_snapshot.running = true;
    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = imu.yaw_rate_dps;
    return WHEEL_HEADING_CONTROL_OK;
}

static void wheel_heading_fault(wheel_heading_control_result_t result,
    car_control_block_reason_t reason)
{
    WheelSpeedControl_Stop(reason);
    g_snapshot.running = false;
    g_snapshot.correction_target_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.last_result = result;
    g_integral_deg_s = 0.0f;
    g_filtered_yaw_rate_dps = 0.0f;
}

static bool wheel_heading_target_is_valid(float target_yaw_deg)
{
    return (target_yaw_deg >= -WHEEL_HEADING_CONTROL_TARGET_MAX_DEG) &&
        (target_yaw_deg <= WHEEL_HEADING_CONTROL_TARGET_MAX_DEG);
}

static bool wheel_heading_speed_is_valid(float speed_pps)
{
    return (speed_pps >= -WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (speed_pps <= WHEEL_SPEED_CONTROL_TARGET_MAX_PPS);
}

static float wheel_heading_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float wheel_heading_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float wheel_heading_clamp(
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

static bool wheel_heading_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
