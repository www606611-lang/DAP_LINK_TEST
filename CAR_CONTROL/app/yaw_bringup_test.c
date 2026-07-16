#include "yaw_bringup_test.h"

#include "icm20948.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define YAW_BRINGUP_DEFAULT_TARGET_DEG       -45.0f
#define YAW_BRINGUP_DEFAULT_OUTPUT_LIMIT     750U
#define YAW_BRINGUP_DEFAULT_TIMEOUT_MS      5000U

static yaw_bringup_test_state_t g_state;
static yaw_bringup_config_t g_config;
static uint32_t g_run_count;
static bool g_start_requested;
static bool g_stop_requested;

static void yaw_bringup_start(uint32_t now_ms);
static void yaw_bringup_stop(car_control_block_reason_t reason,
    yaw_bringup_test_state_t next_state);

void YawBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        YAW_BRINGUP_TEST_LOCKED : YAW_BRINGUP_TEST_READY;
    g_run_count = 0U;
    g_start_requested = false;
    g_stop_requested = false;
    (void) WheelYawControl_GetConfig(&g_config.control);
    g_config.target_yaw_deg = YAW_BRINGUP_DEFAULT_TARGET_DEG;
    g_config.output_limit_permille =
        YAW_BRINGUP_DEFAULT_OUTPUT_LIMIT;
    g_config.timeout_ms = YAW_BRINGUP_DEFAULT_TIMEOUT_MS;
}

void YawBringupTest_Task(uint32_t now_ms)
{
    wheel_yaw_control_snapshot_t yaw;
    bool start_event = g_start_requested;
    bool stop_event = g_stop_requested;

    g_start_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (g_state == YAW_BRINGUP_TEST_RUNNING) {
            yaw_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                YAW_BRINGUP_TEST_ABORTED);
        }
        return;
    }

    switch (g_state) {
        case YAW_BRINGUP_TEST_LOCKED:
            return;

        case YAW_BRINGUP_TEST_READY:
        case YAW_BRINGUP_TEST_COMPLETE:
        case YAW_BRINGUP_TEST_ABORTED:
            if (start_event) {
                yaw_bringup_start(now_ms);
            }
            return;

        case YAW_BRINGUP_TEST_RUNNING:
            if (!WheelYawControl_GetSnapshot(&yaw)) {
                yaw_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                    YAW_BRINGUP_TEST_ABORTED);
                return;
            }
            if (!yaw.running) {
                if (yaw.settled &&
                    (yaw.last_result == WHEEL_YAW_CONTROL_OK) &&
                    (ControlSupervisor_GetBlockReason() ==
                        CAR_CONTROL_BLOCK_TEST_COMPLETE)) {
                    g_state = YAW_BRINGUP_TEST_COMPLETE;
                } else {
                    g_state = YAW_BRINGUP_TEST_ABORTED;
                }
            }
            return;

        default:
            yaw_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                YAW_BRINGUP_TEST_ABORTED);
            return;
    }
}

yaw_bringup_config_result_t YawBringupTest_SetConfig(
    const yaw_bringup_config_t *config)
{
    if (config == NULL) {
        return YAW_BRINGUP_CONFIG_BAD_ARGUMENT;
    }
    if (g_state == YAW_BRINGUP_TEST_RUNNING) {
        return YAW_BRINGUP_CONFIG_BUSY;
    }
    if (!WheelYawControl_ConfigIsValid(&config->control) ||
        !(config->target_yaw_deg >=
            -WHEEL_YAW_CONTROL_TARGET_MAX_DEG) ||
        !(config->target_yaw_deg <=
            WHEEL_YAW_CONTROL_TARGET_MAX_DEG) ||
        (config->target_yaw_deg == 0.0f) ||
        (config->output_limit_permille < YAW_BRINGUP_OUTPUT_MIN) ||
        (config->output_limit_permille > YAW_BRINGUP_OUTPUT_MAX) ||
        (config->timeout_ms < WHEEL_YAW_CONTROL_RUN_MIN_MS) ||
        (config->timeout_ms > WHEEL_YAW_CONTROL_RUN_MAX_MS)) {
        return YAW_BRINGUP_CONFIG_BAD_RANGE;
    }
    if (WheelYawControl_SetConfig(&config->control) !=
        WHEEL_YAW_CONTROL_OK) {
        return YAW_BRINGUP_CONFIG_BUSY;
    }
    g_config = *config;
    return YAW_BRINGUP_CONFIG_OK;
}

bool YawBringupTest_GetConfig(yaw_bringup_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

bool YawBringupTest_RequestStart(void)
{
    if ((g_state == YAW_BRINGUP_TEST_LOCKED) ||
        (g_state == YAW_BRINGUP_TEST_RUNNING)) {
        return false;
    }
    g_start_requested = true;
    return true;
}

void YawBringupTest_RequestStop(void)
{
    g_stop_requested = true;
}

yaw_bringup_test_state_t YawBringupTest_GetState(void)
{
    return g_state;
}

const char *YawBringupTest_GetStateText(void)
{
    switch (g_state) {
        case YAW_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case YAW_BRINGUP_TEST_READY:
            return "READY";
        case YAW_BRINGUP_TEST_RUNNING:
            return "RUN";
        case YAW_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case YAW_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

uint32_t YawBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void yaw_bringup_start(uint32_t now_ms)
{
    icm20948_snapshot_t imu;

    if (!ICM20948_GetSnapshot(&imu) || !imu.ready ||
        !imu.attitude_valid || !imu.stationary) {
        g_state = YAW_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelYawControl_SetConfig(&g_config.control) !=
        WHEEL_YAW_CONTROL_OK) {
        g_state = YAW_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = YAW_BRINGUP_TEST_ABORTED;
        return;
    }

    ICM20948_ResetYaw();
    if (WheelYawControl_StartRelative(
            g_config.target_yaw_deg,
            g_config.timeout_ms, now_ms) != WHEEL_YAW_CONTROL_OK) {
        g_state = YAW_BRINGUP_TEST_ABORTED;
        return;
    }
    g_run_count++;
    g_state = YAW_BRINGUP_TEST_RUNNING;
}

static void yaw_bringup_stop(car_control_block_reason_t reason,
    yaw_bringup_test_state_t next_state)
{
    WheelYawControl_Stop(reason);
    g_state = next_state;
}
