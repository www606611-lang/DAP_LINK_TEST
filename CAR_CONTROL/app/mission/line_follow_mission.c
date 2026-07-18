#include "line_follow_mission.h"

#include "board_motor_safe.h"
#include "control_supervisor.h"
#include "line_sensor.h"
#include "runtime_metrics.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define LINE_FOLLOW_MISSION_KP                 30.0f
#define LINE_FOLLOW_MISSION_KI                  0.0f
#define LINE_FOLLOW_MISSION_KD                  0.0f
#define LINE_FOLLOW_MISSION_MAX_CORRECTION_PPS 900.0f
#define LINE_FOLLOW_MISSION_DEADBAND             2.0f

static line_follow_mission_snapshot_t g_snapshot;
static bool g_start_requested;
static bool g_stop_requested;
static uint32_t g_started_ms;

static bool line_follow_mission_can_start(void);
static void line_follow_mission_start(uint32_t now_ms);
static void line_follow_mission_stop(void);
static void line_follow_mission_fault(
    wheel_line_tracking_result_t result,
    car_control_block_reason_t reason);

void LineFollowMission_Init(bool reset_locked)
{
    g_snapshot.state = reset_locked ?
        LINE_FOLLOW_MISSION_LOCKED : LINE_FOLLOW_MISSION_READY;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    g_snapshot.base_speed_pps = LINE_FOLLOW_MISSION_BASE_SPEED_PPS;
    g_snapshot.output_limit_permille =
        LINE_FOLLOW_MISSION_OUTPUT_LIMIT_PERMILLE;
    g_snapshot.run_count = 0U;
    g_snapshot.elapsed_ms = 0U;
    g_start_requested = false;
    g_stop_requested = false;
    g_started_ms = 0U;
}

void LineFollowMission_Task(uint32_t now_ms)
{
    line_sensor_snapshot_t sensor;
    wheel_line_tracking_snapshot_t control;
    wheel_line_tracking_result_t result;
    bool start_event = g_start_requested;
    bool stop_event = g_stop_requested;

    g_start_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (LineFollowMission_IsActive()) {
            line_follow_mission_stop();
        }
        return;
    }

    if (!LineFollowMission_IsActive()) {
        if (start_event) {
            line_follow_mission_start(now_ms);
        }
        return;
    }

    g_snapshot.elapsed_ms = now_ms - g_started_ms;
    if (!WheelLineTrackingControl_GetSnapshot(&control)) {
        result = WHEEL_LINE_TRACKING_NOT_RUNNING;
        line_follow_mission_fault(result,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!control.running) {
        result = (control.last_result == WHEEL_LINE_TRACKING_OK) ?
            WHEEL_LINE_TRACKING_NOT_RUNNING : control.last_result;
        line_follow_mission_fault(result,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!LineSensor_GetSnapshot(&sensor) || !sensor.ready) {
        line_follow_mission_fault(WHEEL_LINE_TRACKING_SENSOR_STALE,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    result = WheelLineTrackingControl_SetCommand(
        g_snapshot.base_speed_pps,
        sensor.line_error, sensor.active_count, sensor.line_seen,
        sensor.last_sample_ms, now_ms);
    if (result != WHEEL_LINE_TRACKING_OK) {
        line_follow_mission_fault(result,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
    }
}

bool LineFollowMission_RequestStart(void)
{
    wheel_line_tracking_snapshot_t control;

    if ((g_snapshot.state == LINE_FOLLOW_MISSION_LOCKED) ||
        LineFollowMission_IsActive() || g_start_requested ||
        g_stop_requested ||
        !line_follow_mission_can_start() ||
        !WheelLineTrackingControl_GetSnapshot(&control) ||
        control.running) {
        return false;
    }
    g_start_requested = true;
    return true;
}

void LineFollowMission_RequestStop(void)
{
    g_stop_requested = true;
}

bool LineFollowMission_IsActive(void)
{
    return g_snapshot.state == LINE_FOLLOW_MISSION_RUNNING;
}

const char *LineFollowMission_GetStateText(void)
{
    switch (g_snapshot.state) {
        case LINE_FOLLOW_MISSION_LOCKED:
            return "LOCKED";
        case LINE_FOLLOW_MISSION_READY:
            return "READY";
        case LINE_FOLLOW_MISSION_RUNNING:
            return "RUN";
        case LINE_FOLLOW_MISSION_STOPPED:
            return "STOP";
        case LINE_FOLLOW_MISSION_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

bool LineFollowMission_GetSnapshot(
    line_follow_mission_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static bool line_follow_mission_can_start(void)
{
    return BoardMotorSafe_IsHighImpedance() &&
        (ControlSupervisor_GetMode() == CAR_CONTROL_MODE_SAFE_IDLE);
}

static void line_follow_mission_start(uint32_t now_ms)
{
    line_sensor_snapshot_t sensor;
    wheel_line_tracking_config_t config;
    wheel_line_tracking_result_t result;

    if (!line_follow_mission_can_start()) {
        line_follow_mission_fault(WHEEL_LINE_TRACKING_BUSY,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!LineSensor_GetSnapshot(&sensor)) {
        line_follow_mission_fault(WHEEL_LINE_TRACKING_SENSOR_STALE,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (!sensor.ready || !sensor.line_seen ||
        ((uint32_t) (now_ms - sensor.last_sample_ms) >
            WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS)) {
        line_follow_mission_fault(
            sensor.line_seen ? WHEEL_LINE_TRACKING_SENSOR_STALE :
                WHEEL_LINE_TRACKING_LINE_LOST,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    config.kp = LINE_FOLLOW_MISSION_KP;
    config.ki = LINE_FOLLOW_MISSION_KI;
    config.kd = LINE_FOLLOW_MISSION_KD;
    config.max_correction_pps =
        LINE_FOLLOW_MISSION_MAX_CORRECTION_PPS;
    config.deadband = LINE_FOLLOW_MISSION_DEADBAND;
    result = WheelLineTrackingControl_SetConfig(&config);
    if (result != WHEEL_LINE_TRACKING_OK) {
        line_follow_mission_fault(result,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_snapshot.output_limit_permille,
            g_snapshot.output_limit_permille) !=
        WHEEL_SPEED_CONTROL_OK) {
        line_follow_mission_fault(WHEEL_LINE_TRACKING_SPEED_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    AppRuntimeMetrics_Reset(now_ms);
    result = WheelLineTrackingControl_Start(
        g_snapshot.base_speed_pps,
        sensor.line_error, sensor.active_count, sensor.line_seen,
        sensor.last_sample_ms, now_ms);
    if (result != WHEEL_LINE_TRACKING_OK) {
        line_follow_mission_fault(result,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    g_started_ms = now_ms;
    g_snapshot.elapsed_ms = 0U;
    g_snapshot.last_result = WHEEL_LINE_TRACKING_OK;
    g_snapshot.run_count++;
    g_snapshot.state = LINE_FOLLOW_MISSION_RUNNING;
}

static void line_follow_mission_stop(void)
{
    WheelLineTrackingControl_Stop(CAR_CONTROL_BLOCK_OPERATOR_STOP);
    g_snapshot.last_result = WHEEL_LINE_TRACKING_STOPPED;
    g_snapshot.state = LINE_FOLLOW_MISSION_STOPPED;
}

static void line_follow_mission_fault(
    wheel_line_tracking_result_t result,
    car_control_block_reason_t reason)
{
    WheelLineTrackingControl_Stop(reason);
    ControlSupervisor_EmergencyStop(reason);
    g_snapshot.last_result = result;
    g_snapshot.state = LINE_FOLLOW_MISSION_FAULT;
}
