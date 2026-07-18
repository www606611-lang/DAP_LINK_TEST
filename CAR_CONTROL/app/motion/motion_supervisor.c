#include "motion_supervisor.h"

#include "board_motor_safe.h"
#include "icm20948.h"
#include "wheel_odometry.h"
#include "wheel_speed_control.h"

#include <math.h>
#include <stddef.h>

#define MOTION_SUPERVISOR_UPDATE_INTERVAL_MS 10U
#define MOTION_SUPERVISOR_IMU_MAX_AGE_MS     50U
#define MOTION_SUPERVISOR_TIMEOUT_MIN_MS    500U
#define MOTION_SUPERVISOR_TIMEOUT_MAX_MS  60000U
#define MOTION_SUPERVISOR_MAX_DELTA_COUNT 100000000L
#define MOTION_SUPERVISOR_SPEED_MIN_PPS    100.0f
#define MOTION_SUPERVISOR_POSITION_KP         3.0f
#define MOTION_SUPERVISOR_HEADING_KP        30.0f
#define MOTION_SUPERVISOR_HEADING_MAX_PPS  400.0f
#define MOTION_SUPERVISOR_POSITION_TOLERANCE 24
#define MOTION_SUPERVISOR_HEADING_TOLERANCE   1.0f
#define MOTION_SUPERVISOR_SETTLE_TIME_MS    200U
#define MOTION_SUPERVISOR_HEADING_DEADBAND   0.5f

static motion_supervisor_snapshot_t g_snapshot;
static int32_t g_target_count;
static int32_t g_start_count;
static float g_max_speed_pps;
static uint32_t g_timeout_ms;
static uint32_t g_started_ms;
static uint32_t g_last_update_ms;
static uint32_t g_last_command_ms;
static uint32_t g_settle_started_ms;
static bool g_start_requested;
static bool g_stop_requested;
static bool g_pending_hold_heading;
static int32_t g_pending_delta_count;
static float g_pending_heading_deg;
static float g_pending_max_speed_pps;
static uint32_t g_pending_timeout_ms;

static bool motion_supervisor_float_is_valid(float value);
static float motion_supervisor_wrap_deg(float angle_deg);
static float motion_supervisor_clamp(
    float value, float minimum, float maximum);
static int32_t motion_supervisor_abs_i32(int32_t value);
static void motion_supervisor_start(uint32_t now_ms);
static void motion_supervisor_complete(void);
static void motion_supervisor_fault(
    motion_supervisor_result_t result,
    car_control_block_reason_t reason);

void MotionSupervisor_Init(bool reset_locked)
{
    g_snapshot.state = reset_locked ? MOTION_SUPERVISOR_LOCKED :
        MOTION_SUPERVISOR_READY;
    g_snapshot.last_result = MOTION_SUPERVISOR_OK;
    g_snapshot.target_delta_count = 0;
    g_snapshot.target_count = 0;
    g_snapshot.current_count = 0;
    g_snapshot.position_error_count = 0;
    g_snapshot.target_heading_deg = 0.0f;
    g_snapshot.current_heading_deg = 0.0f;
    g_snapshot.heading_error_deg = 0.0f;
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.heading_correction_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.run_count = 0U;
    g_snapshot.update_count = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.output_limit_permille =
        MOTION_SUPERVISOR_DEFAULT_OUTPUT_LIMIT_PERMILLE;
    g_snapshot.hold_heading = false;
    g_snapshot.settled = false;
    g_snapshot.running = false;
    g_target_count = 0;
    g_start_count = 0;
    g_max_speed_pps = 0.0f;
    g_timeout_ms = 0U;
    g_started_ms = 0U;
    g_last_update_ms = 0U;
    g_last_command_ms = 0U;
    g_settle_started_ms = 0U;
    g_start_requested = false;
    g_stop_requested = false;
    g_pending_hold_heading = false;
    g_pending_delta_count = 0;
    g_pending_heading_deg = 0.0f;
    g_pending_max_speed_pps = 0.0f;
    g_pending_timeout_ms = 0U;
}

void MotionSupervisor_Task(uint32_t now_ms)
{
    wheel_odometry_snapshot_t odometry;
    icm20948_snapshot_t imu;
    wheel_speed_control_snapshot_t speed;
    int32_t position_error;
    float position_speed;
    float heading_error;
    float correction;
    float left_target;
    float right_target;
    uint32_t elapsed_ms;

    if (g_stop_requested) {
        g_stop_requested = false;
        if (g_snapshot.running) {
            WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_OPERATOR_STOP);
            g_snapshot.last_result = MOTION_SUPERVISOR_STOPPED;
            g_snapshot.state = MOTION_SUPERVISOR_ABORTED;
            g_snapshot.running = false;
        }
        return;
    }
    if (!g_snapshot.running) {
        if (g_start_requested) {
            g_start_requested = false;
            motion_supervisor_start(now_ms);
        }
        return;
    }

    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    g_snapshot.command_age_ms = now_ms - g_last_command_ms;
    if ((uint32_t) (now_ms - g_started_ms) >= g_timeout_ms) {
        motion_supervisor_fault(MOTION_SUPERVISOR_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (!WheelSpeedControl_GetSnapshot(&speed) || !speed.running ||
        (speed.owner_mode != CAR_CONTROL_MODE_MOTION) ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_MOTION)) {
        motion_supervisor_fault(MOTION_SUPERVISOR_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if ((uint32_t) (now_ms - g_last_update_ms) <
        MOTION_SUPERVISOR_UPDATE_INTERVAL_MS) {
        return;
    }
    if (!WheelOdometry_GetSnapshot(&odometry) || !odometry.ready) {
        motion_supervisor_fault(MOTION_SUPERVISOR_SENSOR_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            MOTION_SUPERVISOR_IMU_MAX_AGE_MS)) {
        motion_supervisor_fault(MOTION_SUPERVISOR_IMU_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_snapshot.last_interval_ms = elapsed_ms;
    if (elapsed_ms > g_snapshot.max_interval_ms) {
        g_snapshot.max_interval_ms = elapsed_ms;
    }
    g_snapshot.current_count = odometry.average_count;
    position_error = g_target_count - odometry.average_count;
    g_snapshot.position_error_count = position_error;
    g_snapshot.current_heading_deg = imu.yaw_deg;
    heading_error = motion_supervisor_wrap_deg(
        g_snapshot.target_heading_deg - imu.yaw_deg);
    g_snapshot.heading_error_deg = heading_error;

    if ((motion_supervisor_abs_i32(position_error) <=
            MOTION_SUPERVISOR_POSITION_TOLERANCE) &&
        (fabsf(heading_error) <= MOTION_SUPERVISOR_HEADING_TOLERANCE)) {
        if (!g_snapshot.settled) {
            g_snapshot.settled = true;
            g_settle_started_ms = now_ms;
        }
        if ((uint32_t) (now_ms - g_settle_started_ms) >=
                MOTION_SUPERVISOR_SETTLE_TIME_MS) {
            motion_supervisor_complete();
            return;
        }
        (void) WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms);
        g_snapshot.base_speed_target_pps = 0.0f;
        g_snapshot.heading_correction_pps = 0.0f;
        g_snapshot.left_speed_target_pps = 0.0f;
        g_snapshot.right_speed_target_pps = 0.0f;
        return;
    }

    g_snapshot.settled = false;
    position_speed = (motion_supervisor_abs_i32(position_error) <=
            MOTION_SUPERVISOR_POSITION_TOLERANCE) ? 0.0f :
        MOTION_SUPERVISOR_POSITION_KP * (float) position_error;
    if (position_speed > g_max_speed_pps) {
        position_speed = g_max_speed_pps;
    } else if (position_speed < -g_max_speed_pps) {
        position_speed = -g_max_speed_pps;
    }
    if ((position_speed > 0.0f) &&
        (position_speed < MOTION_SUPERVISOR_SPEED_MIN_PPS)) {
        position_speed = MOTION_SUPERVISOR_SPEED_MIN_PPS;
    } else if ((position_speed < 0.0f) &&
        (position_speed > -MOTION_SUPERVISOR_SPEED_MIN_PPS)) {
        position_speed = -MOTION_SUPERVISOR_SPEED_MIN_PPS;
    }
    correction = MOTION_SUPERVISOR_HEADING_KP * heading_error;
    correction = motion_supervisor_clamp(correction,
        -MOTION_SUPERVISOR_HEADING_MAX_PPS,
        MOTION_SUPERVISOR_HEADING_MAX_PPS);
    if (fabsf(heading_error) <= MOTION_SUPERVISOR_HEADING_DEADBAND) {
        correction = 0.0f;
    }
    left_target = motion_supervisor_clamp(position_speed - correction,
        -WHEEL_SPEED_CONTROL_TARGET_MAX_PPS,
        WHEEL_SPEED_CONTROL_TARGET_MAX_PPS);
    right_target = motion_supervisor_clamp(position_speed + correction,
        -WHEEL_SPEED_CONTROL_TARGET_MAX_PPS,
        WHEEL_SPEED_CONTROL_TARGET_MAX_PPS);
    if (WheelSpeedControl_SetTargets(
            left_target, right_target, now_ms) != WHEEL_SPEED_CONTROL_OK) {
        motion_supervisor_fault(MOTION_SUPERVISOR_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    g_snapshot.base_speed_target_pps = position_speed;
    g_snapshot.heading_correction_pps = correction;
    g_snapshot.left_speed_target_pps = left_target;
    g_snapshot.right_speed_target_pps = right_target;
    g_snapshot.update_count++;
}

bool MotionSupervisor_RequestRelative(
    int32_t delta_count, float target_heading_deg,
    float max_speed_pps, uint32_t timeout_ms)
{
    if ((g_snapshot.state == MOTION_SUPERVISOR_LOCKED) ||
        g_snapshot.running || g_start_requested || g_stop_requested) {
        return false;
    }
    if ((delta_count == 0) ||
        (delta_count < -MOTION_SUPERVISOR_MAX_DELTA_COUNT) ||
        (delta_count > MOTION_SUPERVISOR_MAX_DELTA_COUNT) ||
        !motion_supervisor_float_is_valid(target_heading_deg) ||
        !motion_supervisor_float_is_valid(max_speed_pps) ||
        (max_speed_pps < MOTION_SUPERVISOR_SPEED_MIN_PPS) ||
        (max_speed_pps > WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) ||
        (timeout_ms < MOTION_SUPERVISOR_TIMEOUT_MIN_MS) ||
        (timeout_ms > MOTION_SUPERVISOR_TIMEOUT_MAX_MS) ||
        !BoardMotorSafe_IsHighImpedance() ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_SAFE_IDLE)) {
        return false;
    }
    g_pending_delta_count = delta_count;
    g_pending_heading_deg = target_heading_deg;
    g_pending_max_speed_pps = max_speed_pps;
    g_pending_timeout_ms = timeout_ms;
    g_pending_hold_heading = false;
    g_start_requested = true;
    return true;
}

bool MotionSupervisor_RequestRelativeHoldHeading(
    int32_t delta_count, float max_speed_pps, uint32_t timeout_ms)
{
    if ((g_snapshot.state == MOTION_SUPERVISOR_LOCKED) ||
        g_snapshot.running || g_start_requested || g_stop_requested) {
        return false;
    }
    if ((delta_count == 0) ||
        (delta_count < -MOTION_SUPERVISOR_MAX_DELTA_COUNT) ||
        (delta_count > MOTION_SUPERVISOR_MAX_DELTA_COUNT) ||
        !motion_supervisor_float_is_valid(max_speed_pps) ||
        (max_speed_pps < MOTION_SUPERVISOR_SPEED_MIN_PPS) ||
        (max_speed_pps > WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) ||
        (timeout_ms < MOTION_SUPERVISOR_TIMEOUT_MIN_MS) ||
        (timeout_ms > MOTION_SUPERVISOR_TIMEOUT_MAX_MS) ||
        !BoardMotorSafe_IsHighImpedance() ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_SAFE_IDLE)) {
        return false;
    }
    g_pending_delta_count = delta_count;
    g_pending_heading_deg = 0.0f;
    g_pending_max_speed_pps = max_speed_pps;
    g_pending_timeout_ms = timeout_ms;
    g_pending_hold_heading = true;
    g_start_requested = true;
    return true;
}

void MotionSupervisor_RequestStop(void)
{
    g_stop_requested = true;
}

bool MotionSupervisor_IsActive(void)
{
    return g_snapshot.running;
}

const char *MotionSupervisor_GetStateText(void)
{
    switch (g_snapshot.state) {
        case MOTION_SUPERVISOR_READY:
            return "READY";
        case MOTION_SUPERVISOR_RUNNING:
            return "RUN";
        case MOTION_SUPERVISOR_COMPLETE:
            return "DONE";
        case MOTION_SUPERVISOR_ABORTED:
            return "ABORT";
        case MOTION_SUPERVISOR_LOCKED:
            return "LOCKED";
        default:
            return "UNKNOWN";
    }
}

bool MotionSupervisor_GetSnapshot(
    motion_supervisor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static void motion_supervisor_start(uint32_t now_ms)
{
    wheel_odometry_snapshot_t odometry;
    icm20948_snapshot_t imu;
    wheel_speed_control_result_t speed_result;

    if (!WheelOdometry_GetSnapshot(&odometry) || !odometry.ready ||
        !ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid ||
        ((uint32_t) (now_ms - imu.last_sample_ms) >
            MOTION_SUPERVISOR_IMU_MAX_AGE_MS)) {
        motion_supervisor_fault(MOTION_SUPERVISOR_SENSOR_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    speed_result = WheelSpeedControl_StartForMode(
        CAR_CONTROL_MODE_MOTION, now_ms);
    if (speed_result != WHEEL_SPEED_CONTROL_OK ||
        WheelSpeedControl_SetOutputLimits(
            g_snapshot.output_limit_permille,
            g_snapshot.output_limit_permille) != WHEEL_SPEED_CONTROL_OK ||
        WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms) !=
            WHEEL_SPEED_CONTROL_OK) {
        motion_supervisor_fault(MOTION_SUPERVISOR_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    g_start_count = odometry.average_count;
    g_target_count = g_start_count + g_pending_delta_count;
    g_snapshot.target_delta_count = g_pending_delta_count;
    g_snapshot.target_count = g_target_count;
    g_snapshot.current_count = g_start_count;
    g_snapshot.position_error_count = g_pending_delta_count;
    g_snapshot.target_heading_deg = g_pending_hold_heading ?
        motion_supervisor_wrap_deg(imu.yaw_deg) :
        motion_supervisor_wrap_deg(g_pending_heading_deg);
    g_snapshot.current_heading_deg = imu.yaw_deg;
    g_snapshot.heading_error_deg = motion_supervisor_wrap_deg(
        g_snapshot.target_heading_deg - imu.yaw_deg);
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.heading_correction_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.command_age_ms = 0U;
    g_snapshot.update_count = 0U;
    g_snapshot.last_interval_ms = 0U;
    g_snapshot.max_interval_ms = 0U;
    g_snapshot.last_result = MOTION_SUPERVISOR_OK;
    g_snapshot.hold_heading = g_pending_hold_heading;
    g_snapshot.settled = false;
    g_snapshot.running = true;
    g_snapshot.state = MOTION_SUPERVISOR_RUNNING;
    g_snapshot.run_count++;
    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_settle_started_ms = now_ms;
    g_max_speed_pps = g_pending_max_speed_pps;
    g_timeout_ms = g_pending_timeout_ms;
}

static void motion_supervisor_complete(void)
{
    WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_TEST_COMPLETE);
    g_snapshot.last_result = MOTION_SUPERVISOR_OK;
    g_snapshot.state = MOTION_SUPERVISOR_COMPLETE;
    g_snapshot.running = false;
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.heading_correction_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
}

static void motion_supervisor_fault(
    motion_supervisor_result_t result,
    car_control_block_reason_t reason)
{
    WheelSpeedControl_Stop(reason);
    g_snapshot.last_result = result;
    g_snapshot.state = MOTION_SUPERVISOR_ABORTED;
    g_snapshot.running = false;
    g_snapshot.base_speed_target_pps = 0.0f;
    g_snapshot.heading_correction_pps = 0.0f;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
}

static bool motion_supervisor_float_is_valid(float value)
{
    return isfinite(value) != 0;
}

static float motion_supervisor_wrap_deg(float angle_deg)
{
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float motion_supervisor_clamp(
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

static int32_t motion_supervisor_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}
