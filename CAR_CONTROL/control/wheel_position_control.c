#include "wheel_position_control.h"

#include "encoder_input.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define WHEEL_POSITION_DEFAULT_KP                3.0f
#define WHEEL_POSITION_DEFAULT_MAX_SPEED_PPS  2000.0f
#define WHEEL_POSITION_DEFAULT_SYNC_KP            2.0f
#define WHEEL_POSITION_DEFAULT_SYNC_MAX_PPS     400.0f
#define WHEEL_POSITION_DEFAULT_TOLERANCE_COUNTS 24U
#define WHEEL_POSITION_DEFAULT_SETTLE_SPEED_PPS 120U
#define WHEEL_POSITION_DEFAULT_SETTLE_TIME_MS   200U
#define WHEEL_POSITION_MIN_SPEED_PPS            100.0f
#define WHEEL_POSITION_TOLERANCE_MAX_COUNTS      200U
#define WHEEL_POSITION_SETTLE_SPEED_MAX_PPS      1000U
#define WHEEL_POSITION_SETTLE_TIME_MIN_MS        20U
#define WHEEL_POSITION_SETTLE_TIME_MAX_MS        2000U
#define WHEEL_POSITION_STALL_SPEED_PPS            40U
#define WHEEL_POSITION_STALL_TIME_MS             300U
#define WHEEL_POSITION_RECOVERY_SPEED_PPS        800.0f
#define WHEEL_POSITION_RECOVERY_MAX_MS           400U

typedef struct {
    uint32_t stall_started_ms;
    uint32_t recovery_deadline_ms;
    bool stall_pending;
    bool recovery_active;
} wheel_position_recovery_t;

static wheel_position_control_config_t g_config;
static wheel_position_control_snapshot_t g_snapshot;
static uint32_t g_started_ms;
static uint32_t g_last_update_ms;
static uint32_t g_deadline_ms;
static uint32_t g_settle_started_ms;
static bool g_settle_pending;
static int32_t g_left_start_count;
static int32_t g_right_start_count;
static int8_t g_sync_direction;
static bool g_sync_straight_move;
static wheel_position_recovery_t g_left_recovery;
static wheel_position_recovery_t g_right_recovery;

static bool wheel_position_target_is_valid(int32_t target_count);
static int32_t wheel_position_error(
    int32_t target_count, int32_t measured_count);
static float wheel_position_speed_target(int32_t error_count);
static float wheel_position_apply_recovery(
    wheel_position_recovery_t *recovery, int32_t error_count,
    int32_t measured_speed_pps, float base_target_pps,
    uint32_t *recovery_count, uint32_t now_ms);
static void wheel_position_apply_sync(void);
static float wheel_position_clamp_speed_for_direction(float speed_pps);
static void wheel_position_reset_recovery(
    wheel_position_recovery_t *recovery, uint32_t now_ms);
static bool wheel_position_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);
static uint32_t wheel_position_abs_i32(int32_t value);
static void wheel_position_update_measurements(
    const encoder_input_snapshot_t *left,
    const encoder_input_snapshot_t *right);
static void wheel_position_complete(uint32_t now_ms);
static void wheel_position_fault(wheel_position_control_result_t result,
    car_control_block_reason_t reason);

void WheelPositionControl_Init(uint32_t now_ms)
{
    g_config.kp = WHEEL_POSITION_DEFAULT_KP;
    g_config.max_speed_pps = WHEEL_POSITION_DEFAULT_MAX_SPEED_PPS;
    g_config.sync_kp = WHEEL_POSITION_DEFAULT_SYNC_KP;
    g_config.sync_max_correction_pps =
        WHEEL_POSITION_DEFAULT_SYNC_MAX_PPS;
    g_config.tolerance_counts =
        WHEEL_POSITION_DEFAULT_TOLERANCE_COUNTS;
    g_config.settle_speed_pps =
        WHEEL_POSITION_DEFAULT_SETTLE_SPEED_PPS;
    g_config.settle_time_ms =
        WHEEL_POSITION_DEFAULT_SETTLE_TIME_MS;

    g_snapshot.left_target_count = 0;
    g_snapshot.right_target_count = 0;
    g_snapshot.left_count = 0;
    g_snapshot.right_count = 0;
    g_snapshot.left_error_count = 0;
    g_snapshot.right_error_count = 0;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.sync_error_count = 0;
    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.left_recovery_count = 0U;
    g_snapshot.right_recovery_count = 0U;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_OK;
    g_snapshot.running = false;
    g_snapshot.settled = false;
    g_snapshot.left_recovery_active = false;
    g_snapshot.right_recovery_active = false;
    g_snapshot.sync_active = false;

    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_deadline_ms = now_ms;
    g_settle_started_ms = now_ms;
    g_settle_pending = false;
    g_left_start_count = 0;
    g_right_start_count = 0;
    g_sync_direction = 0;
    g_sync_straight_move = false;
    wheel_position_reset_recovery(&g_left_recovery, now_ms);
    wheel_position_reset_recovery(&g_right_recovery, now_ms);
}

bool WheelPositionControl_ConfigIsValid(
    const wheel_position_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    return (config->kp > 0.0f) &&
        (config->kp <= WHEEL_POSITION_CONTROL_KP_MAX) &&
        (config->max_speed_pps >= WHEEL_POSITION_MIN_SPEED_PPS) &&
        (config->max_speed_pps <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (config->sync_kp >= 0.0f) &&
        (config->sync_kp <= WHEEL_POSITION_CONTROL_SYNC_KP_MAX) &&
        (config->sync_max_correction_pps >= 0.0f) &&
        (config->sync_max_correction_pps <=
            WHEEL_POSITION_CONTROL_SYNC_MAX_PPS) &&
        (config->tolerance_counts > 0U) &&
        (config->tolerance_counts <=
            WHEEL_POSITION_TOLERANCE_MAX_COUNTS) &&
        (config->settle_speed_pps <=
            WHEEL_POSITION_SETTLE_SPEED_MAX_PPS) &&
        (config->settle_time_ms >=
            WHEEL_POSITION_SETTLE_TIME_MIN_MS) &&
        (config->settle_time_ms <=
            WHEEL_POSITION_SETTLE_TIME_MAX_MS);
}

wheel_position_control_result_t WheelPositionControl_SetConfig(
    const wheel_position_control_config_t *config)
{
    if (!WheelPositionControl_ConfigIsValid(config)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BAD_CONFIG;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BUSY;
        return g_snapshot.last_result;
    }

    g_config = *config;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_OK;
    return WHEEL_POSITION_CONTROL_OK;
}

bool WheelPositionControl_GetConfig(
    wheel_position_control_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

wheel_position_control_result_t WheelPositionControl_StartAbsolute(
    int32_t left_target_count, int32_t right_target_count,
    uint32_t timeout_ms, uint32_t now_ms)
{
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;
    wheel_speed_control_result_t speed_result;
    int64_t left_delta;
    int64_t right_delta;

    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    if (!wheel_position_target_is_valid(left_target_count) ||
        !wheel_position_target_is_valid(right_target_count)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }
    if ((timeout_ms < WHEEL_POSITION_CONTROL_RUN_MIN_MS) ||
        (timeout_ms > WHEEL_POSITION_CONTROL_RUN_MAX_MS)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BAD_TIMEOUT;
        return g_snapshot.last_result;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_ENCODER_ERROR;
        return g_snapshot.last_result;
    }

    speed_result = WheelSpeedControl_StartForMode(
        CAR_CONTROL_MODE_POSITION, now_ms);
    if (speed_result != WHEEL_SPEED_CONTROL_OK) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }
    speed_result = WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms);
    if (speed_result != WHEEL_SPEED_CONTROL_OK) {
        WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_SPEED_ERROR;
        return g_snapshot.last_result;
    }

    g_snapshot.left_target_count = left_target_count;
    g_snapshot.right_target_count = right_target_count;
    g_snapshot.left_count = left.count;
    g_snapshot.right_count = right.count;
    g_snapshot.left_error_count = wheel_position_error(
        left_target_count, left.count);
    g_snapshot.right_error_count = wheel_position_error(
        right_target_count, right.count);
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.sync_error_count = 0;
    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.left_recovery_count = 0U;
    g_snapshot.right_recovery_count = 0U;
    g_snapshot.update_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_OK;
    g_snapshot.running = true;
    g_snapshot.settled = false;
    g_snapshot.left_recovery_active = false;
    g_snapshot.right_recovery_active = false;
    g_snapshot.sync_active = false;

    left_delta = (int64_t) left_target_count - left.count;
    right_delta = (int64_t) right_target_count - right.count;
    g_left_start_count = left.count;
    g_right_start_count = right.count;
    g_sync_straight_move = (left_delta != 0) &&
        (left_delta == right_delta);
    g_sync_direction = g_sync_straight_move ?
        ((left_delta > 0) ? 1 : -1) : 0;

    g_started_ms = now_ms;
    g_last_update_ms = now_ms;
    g_deadline_ms = now_ms + timeout_ms;
    g_settle_started_ms = now_ms;
    g_settle_pending = false;
    wheel_position_reset_recovery(&g_left_recovery, now_ms);
    wheel_position_reset_recovery(&g_right_recovery, now_ms);
    return WHEEL_POSITION_CONTROL_OK;
}

wheel_position_control_result_t WheelPositionControl_StartRelative(
    int32_t left_delta_count, int32_t right_delta_count,
    uint32_t timeout_ms, uint32_t now_ms)
{
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;
    int64_t left_target;
    int64_t right_target;

    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_ENCODER_ERROR;
        return g_snapshot.last_result;
    }

    left_target = (int64_t) left.count + left_delta_count;
    right_target = (int64_t) right.count + right_delta_count;
    if ((left_target < -WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS) ||
        (left_target > WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS) ||
        (right_target < -WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS) ||
        (right_target > WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS)) {
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_BAD_TARGET;
        return g_snapshot.last_result;
    }

    return WheelPositionControl_StartAbsolute(
        (int32_t) left_target, (int32_t) right_target,
        timeout_ms, now_ms);
}

void WheelPositionControl_Task(uint32_t now_ms)
{
    encoder_input_snapshot_t left;
    encoder_input_snapshot_t right;
    wheel_speed_control_snapshot_t speed;
    bool within_position;
    bool within_speed;

    if (!g_snapshot.running) {
        return;
    }
    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    if (wheel_position_deadline_reached(now_ms, g_deadline_ms)) {
        wheel_position_fault(WHEEL_POSITION_CONTROL_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (!WheelSpeedControl_GetSnapshot(&speed) || !speed.running ||
        (speed.owner_mode != CAR_CONTROL_MODE_POSITION) ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_POSITION)) {
        g_snapshot.running = false;
        g_snapshot.settled = false;
        g_snapshot.left_speed_target_pps = 0.0f;
        g_snapshot.right_speed_target_pps = 0.0f;
        g_snapshot.sync_correction_pps = 0.0f;
        g_snapshot.sync_active = false;
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_SPEED_ERROR;
        return;
    }
    if ((uint32_t) (now_ms - g_last_update_ms) <
        WHEEL_POSITION_CONTROL_UPDATE_INTERVAL_MS) {
        return;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right)) {
        wheel_position_fault(WHEEL_POSITION_CONTROL_ENCODER_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    g_last_update_ms = now_ms;
    wheel_position_update_measurements(&left, &right);
    g_snapshot.left_speed_target_pps = wheel_position_speed_target(
        g_snapshot.left_error_count);
    g_snapshot.right_speed_target_pps = wheel_position_speed_target(
        g_snapshot.right_error_count);
    g_snapshot.left_speed_target_pps = wheel_position_apply_recovery(
        &g_left_recovery, g_snapshot.left_error_count,
        left.speed_pps, g_snapshot.left_speed_target_pps,
        &g_snapshot.left_recovery_count, now_ms);
    g_snapshot.right_speed_target_pps = wheel_position_apply_recovery(
        &g_right_recovery, g_snapshot.right_error_count,
        right.speed_pps, g_snapshot.right_speed_target_pps,
        &g_snapshot.right_recovery_count, now_ms);
    wheel_position_apply_sync();
    g_snapshot.left_recovery_active = g_left_recovery.recovery_active;
    g_snapshot.right_recovery_active = g_right_recovery.recovery_active;

    if (WheelSpeedControl_SetTargets(
            g_snapshot.left_speed_target_pps,
            g_snapshot.right_speed_target_pps,
            now_ms) != WHEEL_SPEED_CONTROL_OK) {
        g_snapshot.running = false;
        g_snapshot.settled = false;
        g_snapshot.left_speed_target_pps = 0.0f;
        g_snapshot.right_speed_target_pps = 0.0f;
        g_snapshot.sync_correction_pps = 0.0f;
        g_snapshot.sync_active = false;
        g_snapshot.last_result = WHEEL_POSITION_CONTROL_SPEED_ERROR;
        return;
    }

    within_position =
        (wheel_position_abs_i32(g_snapshot.left_error_count) <=
            g_config.tolerance_counts) &&
        (wheel_position_abs_i32(g_snapshot.right_error_count) <=
            g_config.tolerance_counts);
    within_speed =
        (wheel_position_abs_i32(left.speed_pps) <=
            g_config.settle_speed_pps) &&
        (wheel_position_abs_i32(right.speed_pps) <=
            g_config.settle_speed_pps);

    if (within_position && within_speed) {
        if (!g_settle_pending) {
            g_settle_pending = true;
            g_settle_started_ms = now_ms;
        } else if ((uint32_t) (now_ms - g_settle_started_ms) >=
            g_config.settle_time_ms) {
            wheel_position_complete(now_ms);
            return;
        }
    } else {
        g_settle_pending = false;
    }

    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_OK;
}

void WheelPositionControl_Stop(car_control_block_reason_t reason)
{
    if (g_snapshot.running) {
        WheelSpeedControl_Stop(reason);
    }
    g_snapshot.running = false;
    g_snapshot.settled = false;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.left_recovery_active = false;
    g_snapshot.right_recovery_active = false;
    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.sync_active = false;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_STOPPED;
    g_settle_pending = false;
}

bool WheelPositionControl_GetSnapshot(
    wheel_position_control_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static bool wheel_position_target_is_valid(int32_t target_count)
{
    return (target_count >= -WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS) &&
        (target_count <= WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS);
}

static int32_t wheel_position_error(
    int32_t target_count, int32_t measured_count)
{
    int64_t error = (int64_t) target_count - measured_count;

    if (error > INT32_MAX) {
        return INT32_MAX;
    }
    if (error < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t) error;
}

static float wheel_position_speed_target(int32_t error_count)
{
    float speed_target;

    if (wheel_position_abs_i32(error_count) <=
        g_config.tolerance_counts) {
        return 0.0f;
    }

    speed_target = g_config.kp * (float) error_count;
    if (speed_target > g_config.max_speed_pps) {
        return g_config.max_speed_pps;
    }
    if (speed_target < -g_config.max_speed_pps) {
        return -g_config.max_speed_pps;
    }
    return speed_target;
}

static float wheel_position_apply_recovery(
    wheel_position_recovery_t *recovery, int32_t error_count,
    int32_t measured_speed_pps, float base_target_pps,
    uint32_t *recovery_count, uint32_t now_ms)
{
    float recovery_speed = WHEEL_POSITION_RECOVERY_SPEED_PPS;

    if (recovery_speed > g_config.max_speed_pps) {
        recovery_speed = g_config.max_speed_pps;
    }
    if (wheel_position_abs_i32(error_count) <=
        g_config.tolerance_counts) {
        wheel_position_reset_recovery(recovery, now_ms);
        return 0.0f;
    }
    if (wheel_position_abs_i32(measured_speed_pps) >
        WHEEL_POSITION_STALL_SPEED_PPS) {
        wheel_position_reset_recovery(recovery, now_ms);
        return base_target_pps;
    }
    if (recovery->recovery_active) {
        if (!wheel_position_deadline_reached(
                now_ms, recovery->recovery_deadline_ms)) {
            return (error_count > 0) ?
                recovery_speed : -recovery_speed;
        }
        recovery->recovery_active = false;
        recovery->stall_pending = true;
        recovery->stall_started_ms = now_ms;
        return base_target_pps;
    }
    if ((base_target_pps >= recovery_speed) ||
        (base_target_pps <= -recovery_speed)) {
        recovery->stall_pending = false;
        return base_target_pps;
    }
    if (!recovery->stall_pending) {
        recovery->stall_pending = true;
        recovery->stall_started_ms = now_ms;
        return base_target_pps;
    }
    if ((uint32_t) (now_ms - recovery->stall_started_ms) <
        WHEEL_POSITION_STALL_TIME_MS) {
        return base_target_pps;
    }

    recovery->stall_pending = false;
    recovery->recovery_active = true;
    recovery->recovery_deadline_ms = now_ms +
        WHEEL_POSITION_RECOVERY_MAX_MS;
    (*recovery_count)++;
    return (error_count > 0) ? recovery_speed : -recovery_speed;
}

static void wheel_position_apply_sync(void)
{
    float correction;
    float correction_limit = g_config.sync_max_correction_pps;
    bool both_approaching;

    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.sync_active = false;
    if (!g_sync_straight_move || (g_sync_direction == 0) ||
        (g_config.sync_kp <= 0.0f) || (correction_limit <= 0.0f)) {
        return;
    }

    if (g_sync_direction > 0) {
        both_approaching =
            (g_snapshot.left_error_count >
                (int32_t) g_config.tolerance_counts) &&
            (g_snapshot.right_error_count >
                (int32_t) g_config.tolerance_counts);
    } else {
        both_approaching =
            (g_snapshot.left_error_count <
                -(int32_t) g_config.tolerance_counts) &&
            (g_snapshot.right_error_count <
                -(int32_t) g_config.tolerance_counts);
    }
    if (!both_approaching) {
        return;
    }

    if (correction_limit > g_config.max_speed_pps) {
        correction_limit = g_config.max_speed_pps;
    }
    correction = g_config.sync_kp *
        (float) g_snapshot.sync_error_count;
    if (correction > correction_limit) {
        correction = correction_limit;
    } else if (correction < -correction_limit) {
        correction = -correction_limit;
    }

    g_snapshot.left_speed_target_pps =
        wheel_position_clamp_speed_for_direction(
            g_snapshot.left_speed_target_pps - correction);
    g_snapshot.right_speed_target_pps =
        wheel_position_clamp_speed_for_direction(
            g_snapshot.right_speed_target_pps + correction);
    g_snapshot.sync_correction_pps = correction;
    g_snapshot.sync_active = true;
}

static float wheel_position_clamp_speed_for_direction(float speed_pps)
{
    if (g_sync_direction > 0) {
        if (speed_pps < 0.0f) {
            return 0.0f;
        }
        if (speed_pps > g_config.max_speed_pps) {
            return g_config.max_speed_pps;
        }
    } else {
        if (speed_pps > 0.0f) {
            return 0.0f;
        }
        if (speed_pps < -g_config.max_speed_pps) {
            return -g_config.max_speed_pps;
        }
    }
    return speed_pps;
}

static void wheel_position_reset_recovery(
    wheel_position_recovery_t *recovery, uint32_t now_ms)
{
    recovery->stall_started_ms = now_ms;
    recovery->recovery_deadline_ms = now_ms;
    recovery->stall_pending = false;
    recovery->recovery_active = false;
}

static bool wheel_position_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}

static uint32_t wheel_position_abs_i32(int32_t value)
{
    if (value < 0) {
        return (uint32_t) (-(value + 1)) + 1U;
    }
    return (uint32_t) value;
}

static void wheel_position_update_measurements(
    const encoder_input_snapshot_t *left,
    const encoder_input_snapshot_t *right)
{
    g_snapshot.left_count = left->count;
    g_snapshot.right_count = right->count;
    g_snapshot.left_error_count = wheel_position_error(
        g_snapshot.left_target_count, left->count);
    g_snapshot.right_error_count = wheel_position_error(
        g_snapshot.right_target_count, right->count);
    if (g_sync_straight_move) {
        int64_t sync_error =
            ((int64_t) left->count - g_left_start_count) -
            ((int64_t) right->count - g_right_start_count);

        if (sync_error > INT32_MAX) {
            g_snapshot.sync_error_count = INT32_MAX;
        } else if (sync_error < INT32_MIN) {
            g_snapshot.sync_error_count = INT32_MIN;
        } else {
            g_snapshot.sync_error_count = (int32_t) sync_error;
        }
    } else {
        g_snapshot.sync_error_count = 0;
    }
}

static void wheel_position_complete(uint32_t now_ms)
{
    (void) WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms);
    WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_TEST_COMPLETE);
    g_snapshot.running = false;
    g_snapshot.settled = true;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.left_recovery_active = false;
    g_snapshot.right_recovery_active = false;
    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.sync_active = false;
    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_POSITION_CONTROL_OK;
    g_settle_pending = false;
}

static void wheel_position_fault(wheel_position_control_result_t result,
    car_control_block_reason_t reason)
{
    WheelSpeedControl_Stop(reason);
    g_snapshot.running = false;
    g_snapshot.settled = false;
    g_snapshot.left_speed_target_pps = 0.0f;
    g_snapshot.right_speed_target_pps = 0.0f;
    g_snapshot.left_recovery_active = false;
    g_snapshot.right_recovery_active = false;
    g_snapshot.sync_correction_pps = 0.0f;
    g_snapshot.sync_active = false;
    g_snapshot.last_result = result;
    g_settle_pending = false;
}
