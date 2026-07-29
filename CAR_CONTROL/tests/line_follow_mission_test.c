#include "line_follow_mission.h"

#include "board_motor_safe.h"
#include "control_supervisor.h"
#include "line_sensor.h"
#include "runtime_metrics.h"
#include "wheel_speed_control.h"

#include <assert.h>
#include <string.h>

static bool g_high_impedance;
static car_control_mode_t g_control_mode;
static line_sensor_snapshot_t g_sensor;
static wheel_line_tracking_snapshot_t g_control;
static wheel_line_tracking_config_t g_applied_config;
static wheel_line_tracking_result_t g_set_command_result;
static uint16_t g_left_limit;
static uint16_t g_right_limit;
static uint32_t g_start_count;
static uint32_t g_command_count;
static uint32_t g_stop_count;
static uint32_t g_reset_metrics_count;
static uint32_t g_emergency_stop_count;
static car_control_block_reason_t g_stop_reason;
static int16_t g_start_line_error;
static int16_t g_command_line_error;
static uint8_t g_start_active_count;
static uint8_t g_command_active_count;

static void reset_mocks(void)
{
    memset(&g_sensor, 0, sizeof(g_sensor));
    memset(&g_control, 0, sizeof(g_control));
    memset(&g_applied_config, 0, sizeof(g_applied_config));
    g_high_impedance = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_sensor.ready = true;
    g_sensor.line_seen = true;
    g_sensor.active_count = 2U;
    g_sensor.line_error = 1;
    g_sensor.last_sample_ms = 100U;
    g_control.last_result = WHEEL_LINE_TRACKING_OK;
    g_set_command_result = WHEEL_LINE_TRACKING_OK;
    g_left_limit = 0U;
    g_right_limit = 0U;
    g_start_count = 0U;
    g_command_count = 0U;
    g_stop_count = 0U;
    g_reset_metrics_count = 0U;
    g_emergency_stop_count = 0U;
    g_stop_reason = CAR_CONTROL_BLOCK_NONE;
    g_start_line_error = 0;
    g_command_line_error = 0;
    g_start_active_count = 0U;
    g_command_active_count = 0U;
}

static line_follow_mission_snapshot_t mission_snapshot(void)
{
    line_follow_mission_snapshot_t snapshot;

    assert(LineFollowMission_GetSnapshot(&snapshot));
    return snapshot;
}

static void start_mission(void)
{
    assert(LineFollowMission_RequestStart());
    LineFollowMission_Task(100U);
    assert(LineFollowMission_IsActive());
}

static void test_locked_and_busy_start_rejection(void)
{
    reset_mocks();
    LineFollowMission_Init(true);
    assert(!LineFollowMission_RequestStart());

    LineFollowMission_Init(false);
    g_high_impedance = false;
    assert(!LineFollowMission_RequestStart());
    g_high_impedance = true;
    g_control_mode = CAR_CONTROL_MODE_YAW;
    assert(!LineFollowMission_RequestStart());
}

static void test_start_applies_formal_baseline(void)
{
    line_follow_mission_snapshot_t snapshot;

    reset_mocks();
    LineFollowMission_Init(false);
    start_mission();
    snapshot = mission_snapshot();

    assert(snapshot.state == LINE_FOLLOW_MISSION_RUNNING);
    assert(snapshot.run_count == 1U);
    assert(snapshot.base_speed_pps == 1400.0f);
    assert(snapshot.output_limit_permille == 750U);
    assert(g_applied_config.kp == 30.0f);
    assert(g_applied_config.ki == 0.0f);
    assert(g_applied_config.kd == 0.0f);
    assert(g_applied_config.max_correction_pps == 900.0f);
    assert(g_applied_config.deadband == 2.0f);
    assert(g_left_limit == 750U);
    assert(g_right_limit == 750U);
    assert(g_start_count == 1U);
    assert(g_reset_metrics_count == 1U);
    assert(g_start_line_error == g_sensor.line_error);
}

static void test_wide_marker_start_holds_center_until_narrow(void)
{
    line_follow_mission_snapshot_t snapshot;

    reset_mocks();
    LineFollowMission_Init(false);
    g_sensor.active_count = 5U;
    g_sensor.line_error = -15;
    assert(LineFollowMission_RequestStartFromWideMarker(3U));
    LineFollowMission_Task(100U);
    assert(LineFollowMission_IsActive());
    assert(g_start_line_error == 0);
    assert(g_start_active_count == 3U);
    snapshot = mission_snapshot();
    assert(snapshot.centered_start_active);

    g_sensor.active_count = 6U;
    g_sensor.line_error = 12;
    LineFollowMission_Task(110U);
    assert(g_command_line_error == 0);
    assert(g_command_active_count == 3U);
    assert(mission_snapshot().centered_start_active);

    g_sensor.active_count = 4U;
    g_sensor.line_error = -9;
    LineFollowMission_Task(120U);
    assert(g_command_line_error == 0);
    assert(g_command_active_count == 3U);
    assert(mission_snapshot().centered_start_active);

    g_sensor.active_count = 3U;
    g_sensor.line_error = 7;
    LineFollowMission_Task(130U);
    assert(g_command_line_error == 7);
    assert(g_command_active_count == 3U);
    assert(!mission_snapshot().centered_start_active);

    g_sensor.active_count = 6U;
    g_sensor.line_error = -11;
    LineFollowMission_Task(140U);
    assert(g_command_line_error == -11);
    assert(g_command_active_count == 6U);
    assert(!mission_snapshot().centered_start_active);
}

static void test_running_refresh_and_operator_stop(void)
{
    line_follow_mission_snapshot_t snapshot;

    reset_mocks();
    LineFollowMission_Init(false);
    start_mission();
    LineFollowMission_Task(125U);
    snapshot = mission_snapshot();
    assert(g_command_count == 1U);
    assert(snapshot.elapsed_ms == 25U);

    LineFollowMission_RequestStop();
    LineFollowMission_Task(130U);
    snapshot = mission_snapshot();
    assert(snapshot.state == LINE_FOLLOW_MISSION_STOPPED);
    assert(snapshot.last_result == WHEEL_LINE_TRACKING_STOPPED);
    assert(g_stop_count == 1U);
    assert(g_stop_reason == CAR_CONTROL_BLOCK_OPERATOR_STOP);
}

static void test_control_error_faults_and_stops(void)
{
    line_follow_mission_snapshot_t snapshot;

    reset_mocks();
    LineFollowMission_Init(false);
    start_mission();
    g_set_command_result = WHEEL_LINE_TRACKING_LINE_LOST;
    LineFollowMission_Task(120U);
    snapshot = mission_snapshot();

    assert(snapshot.state == LINE_FOLLOW_MISSION_FAULT);
    assert(snapshot.last_result == WHEEL_LINE_TRACKING_LINE_LOST);
    assert(g_stop_reason == CAR_CONTROL_BLOCK_EMERGENCY_STOP);
    assert(g_emergency_stop_count == 1U);
}

int main(void)
{
    test_locked_and_busy_start_rejection();
    test_start_applies_formal_baseline();
    test_wide_marker_start_holds_center_until_narrow();
    test_running_refresh_and_operator_stop();
    test_control_error_faults_and_stops();
    return 0;
}

bool BoardMotorSafe_IsHighImpedance(void)
{
    return g_high_impedance;
}

car_control_mode_t ControlSupervisor_GetMode(void)
{
    return g_control_mode;
}

void ControlSupervisor_EmergencyStop(car_control_block_reason_t reason)
{
    g_emergency_stop_count++;
    g_stop_reason = reason;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_high_impedance = true;
}

bool LineSensor_GetSnapshot(line_sensor_snapshot_t *snapshot)
{
    *snapshot = g_sensor;
    return true;
}

void AppRuntimeMetrics_Reset(uint32_t now_ms)
{
    (void) now_ms;
    g_reset_metrics_count++;
}

wheel_line_tracking_result_t WheelLineTrackingControl_SetConfig(
    const wheel_line_tracking_config_t *config)
{
    g_applied_config = *config;
    return WHEEL_LINE_TRACKING_OK;
}

wheel_speed_control_result_t WheelSpeedControl_SetOutputLimits(
    uint16_t left_permille, uint16_t right_permille)
{
    g_left_limit = left_permille;
    g_right_limit = right_permille;
    return WHEEL_SPEED_CONTROL_OK;
}

wheel_line_tracking_result_t WheelLineTrackingControl_Start(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen, uint32_t observation_ms, uint32_t now_ms)
{
    (void) base_speed_pps;
    (void) line_seen;
    (void) observation_ms;
    (void) now_ms;
    g_start_line_error = line_error;
    g_start_active_count = active_count;
    if ((active_count >= 4U) && (line_error == 0)) {
        return WHEEL_LINE_TRACKING_BAD_COMMAND;
    }
    g_start_count++;
    g_control.running = true;
    return WHEEL_LINE_TRACKING_OK;
}

wheel_line_tracking_result_t WheelLineTrackingControl_SetCommand(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen, uint32_t observation_ms, uint32_t now_ms)
{
    (void) base_speed_pps;
    (void) line_seen;
    (void) observation_ms;
    (void) now_ms;
    g_command_line_error = line_error;
    g_command_active_count = active_count;
    g_command_count++;
    return g_set_command_result;
}

void WheelLineTrackingControl_Stop(car_control_block_reason_t reason)
{
    g_stop_count++;
    g_stop_reason = reason;
    g_control.running = false;
}

bool WheelLineTrackingControl_GetSnapshot(
    wheel_line_tracking_snapshot_t *snapshot)
{
    *snapshot = g_control;
    return true;
}
