#include "heading_bringup_test.h"

#include "icm20948.h"
#include "runtime_metrics.h"
#include "wheel_speed_control.h"

#include <math.h>
#include <stddef.h>

#define HEADING_BRINGUP_DEFAULT_BASE_SPEED_PPS 1200.0f
#define HEADING_BRINGUP_DEFAULT_OUTPUT_LIMIT    650U
#define HEADING_BRINGUP_DEFAULT_DURATION_MS    6000U
#define HEADING_BRINGUP_ARM_SETTLE_MS            40U
#define HEADING_BRINGUP_ARM_TIMEOUT_MS           400U
#define HEADING_BRINGUP_ARM_MAX_RATE_DPS           3.0f

static heading_bringup_test_state_t g_state;
static heading_bringup_config_t g_config;
static uint32_t g_run_count;
static uint32_t g_run_deadline_ms;
static uint32_t g_arm_ready_ms;
static uint32_t g_arm_deadline_ms;
static bool g_start_requested;
static bool g_stop_requested;

static void heading_bringup_arm(uint32_t now_ms);
static void heading_bringup_start(uint32_t now_ms);
static void heading_bringup_stop(car_control_block_reason_t reason,
    heading_bringup_test_state_t next_state);
static float heading_bringup_abs(float value);
static bool heading_bringup_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void HeadingBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        HEADING_BRINGUP_TEST_LOCKED : HEADING_BRINGUP_TEST_READY;
    g_run_count = 0U;
    g_run_deadline_ms = 0U;
    g_arm_ready_ms = 0U;
    g_arm_deadline_ms = 0U;
    g_start_requested = false;
    g_stop_requested = false;
    (void) WheelHeadingControl_GetConfig(&g_config.control);
    g_config.base_speed_pps = HEADING_BRINGUP_DEFAULT_BASE_SPEED_PPS;
    g_config.output_limit_permille =
        HEADING_BRINGUP_DEFAULT_OUTPUT_LIMIT;
    g_config.duration_ms = HEADING_BRINGUP_DEFAULT_DURATION_MS;
}

void HeadingBringupTest_Task(uint32_t now_ms)
{
    wheel_heading_control_snapshot_t heading;
    icm20948_snapshot_t imu;
    bool start_event = g_start_requested;
    bool stop_event = g_stop_requested;

    g_start_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (g_state == HEADING_BRINGUP_TEST_RUNNING) {
            heading_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                HEADING_BRINGUP_TEST_ABORTED);
        } else if (g_state == HEADING_BRINGUP_TEST_ARMING) {
            g_state = HEADING_BRINGUP_TEST_ABORTED;
        }
        return;
    }

    switch (g_state) {
        case HEADING_BRINGUP_TEST_LOCKED:
            return;

        case HEADING_BRINGUP_TEST_READY:
        case HEADING_BRINGUP_TEST_COMPLETE:
        case HEADING_BRINGUP_TEST_ABORTED:
            if (start_event) {
                heading_bringup_arm(now_ms);
            }
            return;

        case HEADING_BRINGUP_TEST_ARMING:
            if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
                !imu.attitude_valid ||
                heading_bringup_deadline_reached(
                    now_ms, g_arm_deadline_ms)) {
                g_state = HEADING_BRINGUP_TEST_ABORTED;
                return;
            }
            if (heading_bringup_deadline_reached(
                    now_ms, g_arm_ready_ms) &&
                (fabsf(imu.data.gz_dps) <=
                    HEADING_BRINGUP_ARM_MAX_RATE_DPS)) {
                heading_bringup_start(now_ms);
            }
            return;

        case HEADING_BRINGUP_TEST_RUNNING:
            if (!WheelHeadingControl_GetSnapshot(&heading) ||
                !heading.running) {
                g_state = HEADING_BRINGUP_TEST_ABORTED;
                return;
            }
            if (heading_bringup_deadline_reached(
                    now_ms, g_run_deadline_ms)) {
                heading_bringup_stop(CAR_CONTROL_BLOCK_TEST_COMPLETE,
                    HEADING_BRINGUP_TEST_COMPLETE);
                return;
            }
            if (WheelHeadingControl_SetCommand(
                    0.0f, g_config.base_speed_pps, now_ms) !=
                WHEEL_HEADING_CONTROL_OK) {
                heading_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                    HEADING_BRINGUP_TEST_ABORTED);
            }
            return;

        default:
            heading_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                HEADING_BRINGUP_TEST_ABORTED);
            return;
    }
}

heading_bringup_config_result_t HeadingBringupTest_SetConfig(
    const heading_bringup_config_t *config)
{
    float base_magnitude;

    if (config == NULL) {
        return HEADING_BRINGUP_CONFIG_BAD_ARGUMENT;
    }
    if (HeadingBringupTest_IsActive()) {
        return HEADING_BRINGUP_CONFIG_BUSY;
    }
    base_magnitude = heading_bringup_abs(config->base_speed_pps);
    if (!WheelHeadingControl_ConfigIsValid(&config->control) ||
        (base_magnitude < HEADING_BRINGUP_BASE_SPEED_MIN_PPS) ||
        ((base_magnitude + config->control.max_correction_pps) >
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) ||
        (config->output_limit_permille < HEADING_BRINGUP_OUTPUT_MIN) ||
        (config->output_limit_permille > HEADING_BRINGUP_OUTPUT_MAX) ||
        (config->duration_ms < HEADING_BRINGUP_DURATION_MIN_MS) ||
        (config->duration_ms > HEADING_BRINGUP_DURATION_MAX_MS)) {
        return HEADING_BRINGUP_CONFIG_BAD_RANGE;
    }
    if (WheelHeadingControl_SetConfig(&config->control) !=
        WHEEL_HEADING_CONTROL_OK) {
        return HEADING_BRINGUP_CONFIG_BUSY;
    }
    g_config = *config;
    return HEADING_BRINGUP_CONFIG_OK;
}

bool HeadingBringupTest_GetConfig(heading_bringup_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

bool HeadingBringupTest_RequestStart(void)
{
    if ((g_state == HEADING_BRINGUP_TEST_LOCKED) ||
        HeadingBringupTest_IsActive() || g_start_requested) {
        return false;
    }
    g_start_requested = true;
    return true;
}

void HeadingBringupTest_RequestStop(void)
{
    g_stop_requested = true;
}

heading_bringup_test_state_t HeadingBringupTest_GetState(void)
{
    return g_state;
}

bool HeadingBringupTest_IsActive(void)
{
    return (g_state == HEADING_BRINGUP_TEST_ARMING) ||
        (g_state == HEADING_BRINGUP_TEST_RUNNING);
}

const char *HeadingBringupTest_GetStateText(void)
{
    switch (g_state) {
        case HEADING_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case HEADING_BRINGUP_TEST_READY:
            return "READY";
        case HEADING_BRINGUP_TEST_ARMING:
            return "ARM";
        case HEADING_BRINGUP_TEST_RUNNING:
            return "RUN";
        case HEADING_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case HEADING_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

uint32_t HeadingBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void heading_bringup_arm(uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid) {
        g_state = HEADING_BRINGUP_TEST_ABORTED;
        return;
    }
    g_arm_ready_ms = now_ms + HEADING_BRINGUP_ARM_SETTLE_MS;
    g_arm_deadline_ms = now_ms + HEADING_BRINGUP_ARM_TIMEOUT_MS;
    g_state = HEADING_BRINGUP_TEST_ARMING;
}

static void heading_bringup_start(uint32_t now_ms)
{
    if (WheelHeadingControl_SetConfig(&g_config.control) !=
        WHEEL_HEADING_CONTROL_OK) {
        g_state = HEADING_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = HEADING_BRINGUP_TEST_ABORTED;
        return;
    }

    AppRuntimeMetrics_Reset(now_ms);
    ICM20948_ResetTimingStats();
    ICM20948_ResetYaw();
    if (WheelHeadingControl_StartHoldCurrent(
            g_config.base_speed_pps, now_ms) !=
        WHEEL_HEADING_CONTROL_OK) {
        g_state = HEADING_BRINGUP_TEST_ABORTED;
        return;
    }
    g_run_deadline_ms = now_ms + g_config.duration_ms;
    g_run_count++;
    g_state = HEADING_BRINGUP_TEST_RUNNING;
}

static void heading_bringup_stop(car_control_block_reason_t reason,
    heading_bringup_test_state_t next_state)
{
    WheelHeadingControl_Stop(reason);
    g_state = next_state;
}

static float heading_bringup_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool heading_bringup_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
