#include "line_tracking_bringup_test.h"

#include "line_tracking_finish_policy.h"
#include "line_sensor.h"
#include "runtime_metrics.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define LINE_TRACKING_BRINGUP_DEFAULT_BASE_SPEED_PPS 1400.0f
#define LINE_TRACKING_BRINGUP_DEFAULT_OUTPUT_LIMIT     750U
#define LINE_TRACKING_BRINGUP_DEFAULT_DURATION_MS    30000U
#define LINE_TRACKING_BRINGUP_FINISH_GRACE_MS         3000U
#define LINE_TRACKING_BRINGUP_FINISH_STABLE_MS         250U
#define LINE_TRACKING_BRINGUP_FINISH_ERROR_MAX           5
#define LINE_TRACKING_BRINGUP_FINISH_COUNT_MAX           2U

static line_tracking_bringup_test_state_t g_state;
static line_tracking_bringup_config_t g_config;
static uint32_t g_run_count;
static line_tracking_finish_policy_t g_finish_policy;
static bool g_start_requested;
static bool g_stop_requested;

static void line_tracking_bringup_start(uint32_t now_ms);
static void line_tracking_bringup_stop(
    car_control_block_reason_t reason,
    line_tracking_bringup_test_state_t next_state);

void LineTrackingBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        LINE_TRACKING_BRINGUP_TEST_LOCKED :
        LINE_TRACKING_BRINGUP_TEST_READY;
    (void) WheelLineTrackingControl_GetConfig(&g_config.control);
    g_config.base_speed_pps =
        LINE_TRACKING_BRINGUP_DEFAULT_BASE_SPEED_PPS;
    g_config.output_limit_permille =
        LINE_TRACKING_BRINGUP_DEFAULT_OUTPUT_LIMIT;
    g_config.duration_ms = LINE_TRACKING_BRINGUP_DEFAULT_DURATION_MS;
    g_run_count = 0U;
    LineTrackingFinishPolicy_Init(
        &g_finish_policy, 0U, LINE_TRACKING_BRINGUP_FINISH_GRACE_MS);
    g_start_requested = false;
    g_stop_requested = false;
}

void LineTrackingBringupTest_Task(uint32_t now_ms)
{
    line_sensor_snapshot_t sensor;
    wheel_line_tracking_snapshot_t control;
    bool start_event = g_start_requested;
    bool stop_event = g_stop_requested;

    g_start_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (g_state == LINE_TRACKING_BRINGUP_TEST_RUNNING) {
            line_tracking_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                LINE_TRACKING_BRINGUP_TEST_ABORTED);
        }
        return;
    }

    switch (g_state) {
        case LINE_TRACKING_BRINGUP_TEST_LOCKED:
            return;

        case LINE_TRACKING_BRINGUP_TEST_READY:
        case LINE_TRACKING_BRINGUP_TEST_COMPLETE:
        case LINE_TRACKING_BRINGUP_TEST_ABORTED:
            if (start_event) {
                line_tracking_bringup_start(now_ms);
            }
            return;

        case LINE_TRACKING_BRINGUP_TEST_RUNNING:
            if (!WheelLineTrackingControl_GetSnapshot(&control) ||
                !control.running) {
                g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
                return;
            }
            if (!LineSensor_GetSnapshot(&sensor) ||
                !sensor.ready) {
                WheelLineTrackingControl_Stop(
                    CAR_CONTROL_BLOCK_EMERGENCY_STOP);
                g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
                return;
            }
            {
                int32_t line_error = sensor.line_error;
                bool centered;
                line_tracking_finish_decision_t finish_decision;

                if (line_error < 0) {
                    line_error = -line_error;
                }
                centered = sensor.line_seen &&
                    (sensor.active_count <=
                        LINE_TRACKING_BRINGUP_FINISH_COUNT_MAX) &&
                    (line_error <=
                        LINE_TRACKING_BRINGUP_FINISH_ERROR_MAX);
                finish_decision = LineTrackingFinishPolicy_Update(
                    &g_finish_policy, now_ms, centered,
                    LINE_TRACKING_BRINGUP_FINISH_STABLE_MS);
                if (finish_decision == LINE_TRACKING_FINISH_COMPLETE) {
                    line_tracking_bringup_stop(
                        CAR_CONTROL_BLOCK_TEST_COMPLETE,
                        LINE_TRACKING_BRINGUP_TEST_COMPLETE);
                    return;
                }
                if (finish_decision == LINE_TRACKING_FINISH_TIMEOUT) {
                    line_tracking_bringup_stop(
                        CAR_CONTROL_BLOCK_COMMAND_TIMEOUT,
                        LINE_TRACKING_BRINGUP_TEST_ABORTED);
                    return;
                }
            }
            if (WheelLineTrackingControl_SetCommand(
                    g_config.base_speed_pps,
                    sensor.line_error, sensor.active_count,
                    sensor.line_seen,
                    sensor.last_sample_ms,
                    now_ms) != WHEEL_LINE_TRACKING_OK) {
                g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
            }
            return;

        default:
            line_tracking_bringup_stop(
                CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                LINE_TRACKING_BRINGUP_TEST_ABORTED);
            return;
    }
}

line_tracking_bringup_config_result_t LineTrackingBringupTest_SetConfig(
    const line_tracking_bringup_config_t *config)
{
    if (config == NULL) {
        return LINE_TRACKING_BRINGUP_CONFIG_BAD_ARGUMENT;
    }
    if (LineTrackingBringupTest_IsActive()) {
        return LINE_TRACKING_BRINGUP_CONFIG_BUSY;
    }
    if (!WheelLineTrackingControl_ConfigIsValid(&config->control) ||
        (config->base_speed_pps <
            LINE_TRACKING_BRINGUP_BASE_SPEED_MIN_PPS) ||
        ((config->base_speed_pps +
            config->control.max_correction_pps) >
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) ||
        (config->output_limit_permille <
            LINE_TRACKING_BRINGUP_OUTPUT_MIN) ||
        (config->output_limit_permille >
            LINE_TRACKING_BRINGUP_OUTPUT_MAX) ||
        (config->duration_ms <
            LINE_TRACKING_BRINGUP_DURATION_MIN_MS) ||
        (config->duration_ms >
            LINE_TRACKING_BRINGUP_DURATION_MAX_MS)) {
        return LINE_TRACKING_BRINGUP_CONFIG_BAD_RANGE;
    }
    if (WheelLineTrackingControl_SetConfig(&config->control) !=
        WHEEL_LINE_TRACKING_OK) {
        return LINE_TRACKING_BRINGUP_CONFIG_BUSY;
    }
    g_config = *config;
    return LINE_TRACKING_BRINGUP_CONFIG_OK;
}

bool LineTrackingBringupTest_GetConfig(
    line_tracking_bringup_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

bool LineTrackingBringupTest_RequestStart(void)
{
    if ((g_state == LINE_TRACKING_BRINGUP_TEST_LOCKED) ||
        LineTrackingBringupTest_IsActive() || g_start_requested) {
        return false;
    }
    g_start_requested = true;
    return true;
}

void LineTrackingBringupTest_RequestStop(void)
{
    g_stop_requested = true;
}

line_tracking_bringup_test_state_t LineTrackingBringupTest_GetState(void)
{
    return g_state;
}

bool LineTrackingBringupTest_IsActive(void)
{
    return g_state == LINE_TRACKING_BRINGUP_TEST_RUNNING;
}

const char *LineTrackingBringupTest_GetStateText(void)
{
    switch (g_state) {
        case LINE_TRACKING_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case LINE_TRACKING_BRINGUP_TEST_READY:
            return "READY";
        case LINE_TRACKING_BRINGUP_TEST_RUNNING:
            return "RUN";
        case LINE_TRACKING_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case LINE_TRACKING_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

uint32_t LineTrackingBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void line_tracking_bringup_start(uint32_t now_ms)
{
    line_sensor_snapshot_t sensor;

    if (!LineSensor_GetSnapshot(&sensor) ||
        !sensor.ready || !sensor.line_seen ||
        ((uint32_t) (now_ms - sensor.last_sample_ms) >
            WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS)) {
        g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelLineTrackingControl_SetConfig(&g_config.control) !=
        WHEEL_LINE_TRACKING_OK) {
        g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
        return;
    }

    AppRuntimeMetrics_Reset(now_ms);
    if (WheelLineTrackingControl_Start(
            g_config.base_speed_pps,
            sensor.line_error, sensor.active_count,
            sensor.line_seen,
            sensor.last_sample_ms,
            now_ms) != WHEEL_LINE_TRACKING_OK) {
        g_state = LINE_TRACKING_BRINGUP_TEST_ABORTED;
        return;
    }
    LineTrackingFinishPolicy_Init(&g_finish_policy,
        now_ms + g_config.duration_ms,
        LINE_TRACKING_BRINGUP_FINISH_GRACE_MS);
    g_run_count++;
    g_state = LINE_TRACKING_BRINGUP_TEST_RUNNING;
}

static void line_tracking_bringup_stop(
    car_control_block_reason_t reason,
    line_tracking_bringup_test_state_t next_state)
{
    WheelLineTrackingControl_Stop(reason);
    g_state = next_state;
}
