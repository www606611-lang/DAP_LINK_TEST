#include "h_mission_runtime.h"

#include "board_motor_safe.h"
#include "control_supervisor.h"
#include "line_follow_mission.h"
#include "line_sensor.h"
#include "motion_supervisor.h"
#include "wheel_line_tracking_control.h"
#include "wheel_odometry.h"
#include <stddef.h>

#define H_MISSION_RUNTIME_MARKER_ACTIVE_MIN 5U
#define H_MISSION_RUNTIME_FINISH_MARKER_ACTIVE_MIN 4U
#define H_MISSION_RUNTIME_START_NARROW_ACTIVE_MAX 3U
#define H_MISSION_RUNTIME_MARKER_CONFIRM_MS 20U
#define H_MISSION_RUNTIME_FINISH_MARKER_CONFIRM_MS 10U
#define H_MISSION_RUNTIME_MARKER_RELEASE_MS 20U
#define H_MISSION_RUNTIME_START_SPEED_PPS  500.0f
#define H_MISSION_RUNTIME_CRUISE_SPEED_PPS 3200.0f
#define H_MISSION_RUNTIME_ACCEL_TIME_MS    3000U
#define H_MISSION_RUNTIME_DECEL_START_COUNT 18000
#define H_MISSION_RUNTIME_FINISH_SPEED_PPS   700.0f
#define H_MISSION_RUNTIME_DECEL_TIME_MS     2400U

static h_mission_t g_mission;
static h_route_events_t g_route;
static h_mission_runtime_snapshot_t g_snapshot;
static bool g_primary_requested;
static bool g_stop_requested;
static bool g_line_owned;
static bool g_precision_owned;
static bool g_executor_fault;
static h_mission_speed_stage_t g_speed_stage;
static float g_requested_base_speed_pps;
static bool g_decel_active;
static uint32_t g_decel_started_ms;
static float g_decel_start_speed_pps;

static void h_mission_runtime_collect_input(uint32_t now_ms,
    h_mission_input_t *input, wheel_odometry_snapshot_t *odometry);
static void h_mission_runtime_update_route(uint32_t now_ms,
    const line_sensor_snapshot_t *sensor,
    const wheel_odometry_snapshot_t *odometry);
static void h_mission_runtime_execute(uint32_t now_ms);
static void h_mission_runtime_stop_owned(void);
static void h_mission_runtime_refresh_snapshot(void);
static float h_mission_runtime_speed_command(uint32_t elapsed_ms);
static float h_mission_runtime_s_curve(float start, float finish,
    uint32_t elapsed_ms, uint32_t duration_ms);

void HMissionRuntime_Init(bool reset_locked, uint32_t now_ms)
{
    const h_route_config_t route_config = {
        .precision_stop_delta_count =
            H_MISSION_ROUTE_PRECISION_DELTA_COUNT,
        .finish_arm_count = H_MISSION_ROUTE_FINISH_ARM_COUNT,
        .finish_rearm_ms = H_MISSION_ROUTE_FINISH_REARM_MS,
        .marker_active_min = H_MISSION_RUNTIME_MARKER_ACTIVE_MIN,
        .finish_marker_active_min =
            H_MISSION_RUNTIME_FINISH_MARKER_ACTIVE_MIN,
        .marker_confirm_ms = H_MISSION_RUNTIME_MARKER_CONFIRM_MS,
        .finish_marker_confirm_ms =
            H_MISSION_RUNTIME_FINISH_MARKER_CONFIRM_MS,
        .marker_release_ms = H_MISSION_RUNTIME_MARKER_RELEASE_MS
    };

    (void) now_ms;
    HMission_Init(&g_mission, reset_locked);
    HRouteEvents_Init(&g_route, &route_config);
    g_primary_requested = false;
    g_stop_requested = false;
    g_line_owned = false;
    g_precision_owned = false;
    g_executor_fault = false;
    g_speed_stage = H_MISSION_SPEED_IDLE;
    g_requested_base_speed_pps = 0.0f;
    g_decel_active = false;
    g_decel_started_ms = 0U;
    g_decel_start_speed_pps = 0.0f;
    g_snapshot.executor_error_count = 0U;
    h_mission_runtime_refresh_snapshot();
}

void HMissionRuntime_Task(uint32_t now_ms)
{
    h_mission_input_t input;
    wheel_odometry_snapshot_t odometry;
    h_mission_state_t old_state = g_mission.snapshot.state;

    h_mission_runtime_collect_input(now_ms, &input, &odometry);
    if (g_primary_requested &&
        ((old_state == H_MISSION_STATE_FINISHED) ||
         (old_state == H_MISSION_STATE_FAULT))) {
        g_executor_fault = false;
        input.chassis_fault = false;
        input.reset_requested = true;
    } else if (g_primary_requested) {
        input.start_pressed = true;
    }
    input.stop_requested = g_stop_requested;
    g_primary_requested = false;
    g_stop_requested = false;

    HMission_Step(&g_mission, &input);
    if ((old_state != H_MISSION_STATE_RUNNING) &&
        (g_mission.snapshot.state == H_MISSION_STATE_RUNNING)) {
        g_decel_active = false;
        g_decel_started_ms = 0U;
        g_decel_start_speed_pps = 0.0f;
        if (!odometry.ready ||
            !HRouteEvents_Start(&g_route, now_ms,
                odometry.average_count)) {
            g_executor_fault = true;
            g_snapshot.executor_error_count++;
        }
    }
    if (!HMission_IsActive(&g_mission)) {
        HRouteEvents_Stop(&g_route);
    }

    h_mission_runtime_execute(now_ms);
    h_mission_runtime_refresh_snapshot();
}

bool HMissionRuntime_SetRouteConfig(const h_route_config_t *config)
{
    if (HMissionRuntime_IsActive() ||
        !BoardMotorSafe_IsHighImpedance() ||
        (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_SAFE_IDLE)) {
        return false;
    }
    return HRouteEvents_SetConfig(&g_route, config);
}

void HMissionRuntime_RequestPrimary(void)
{
    g_primary_requested = true;
}

void HMissionRuntime_RequestStop(void)
{
    g_stop_requested = true;
}

bool HMissionRuntime_IsActive(void)
{
    return HMission_IsActive(&g_mission);
}

bool HMissionRuntime_GetSnapshot(
    h_mission_runtime_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

const char *HMissionRuntime_GetStateText(void)
{
    switch (g_mission.snapshot.state) {
        case H_MISSION_STATE_LOCKED: return "LOCK";
        case H_MISSION_STATE_READY: return "READY";
        case H_MISSION_STATE_ARMED: return "ARMED";
        case H_MISSION_STATE_RUNNING: return "RUN";
        case H_MISSION_STATE_PRECISION_STOP: return "PSTOP";
        case H_MISSION_STATE_FINISHED: return "DONE";
        case H_MISSION_STATE_FAULT: return "FAULT";
        default: return "UNK";
    }
}

const char *HMissionRuntime_GetPhaseText(void)
{
    switch (g_mission.snapshot.phase) {
        case H_MISSION_PHASE_IDLE: return "IDLE";
        case H_MISSION_PHASE_BALL_ONLY: return "BALL";
        case H_MISSION_PHASE_LEAVE_START_A: return "LEAVE-A";
        case H_MISSION_PHASE_RUN_TO_B: return "TO-B";
        case H_MISSION_PHASE_RUN_TO_A: return "LAP";
        case H_MISSION_PHASE_PRECISION_STOP: return "PSTOP";
        default: return "UNK";
    }
}

static void h_mission_runtime_collect_input(uint32_t now_ms,
    h_mission_input_t *input, wheel_odometry_snapshot_t *odometry)
{
    line_sensor_snapshot_t sensor = {0};
    line_follow_mission_snapshot_t line = {0};
    motion_supervisor_snapshot_t motion = {0};
    bool has_sensor;
    bool has_odometry;

    *input = (h_mission_input_t) {0};
    *odometry = (wheel_odometry_snapshot_t) {0};
    has_sensor = LineSensor_GetSnapshot(&sensor);
    has_odometry = WheelOdometry_GetSnapshot(odometry);
    (void) LineFollowMission_GetSnapshot(&line);
    (void) MotionSupervisor_GetSnapshot(&motion);

    h_mission_runtime_update_route(now_ms,
        has_sensor ? &sensor : NULL,
        has_odometry ? odometry : NULL);

    input->now_ms = now_ms;
    input->chassis_high_z = BoardMotorSafe_IsHighImpedance();
    input->chassis_fault = g_executor_fault ||
        (g_precision_owned &&
         (motion.state == MOTION_SUPERVISOR_ABORTED));
    input->line_ready = g_route.snapshot.configuration_valid &&
        g_route.snapshot.marker_wide &&
        has_sensor && sensor.ready && sensor.line_seen &&
        ((uint32_t) (now_ms - sensor.last_sample_ms) <=
            WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS) &&
        has_odometry && odometry->ready;
    input->line_fault = g_line_owned &&
        (g_mission.snapshot.state == H_MISSION_STATE_RUNNING) &&
        (line.state == LINE_FOLLOW_MISSION_FAULT);
    input->left_start_a = g_route.snapshot.left_start_a_event;
    input->b_passed = false;
    input->finish_a_passed =
        g_route.snapshot.finish_a_passed_event;
    if (g_route.config.precision_stop_delta_count == 0) {
        input->precision_stop_complete =
            (g_mission.snapshot.state ==
                H_MISSION_STATE_PRECISION_STOP) &&
            !g_line_owned && !g_precision_owned &&
            input->chassis_high_z;
    } else {
        input->precision_stop_complete = g_precision_owned &&
            (motion.state == MOTION_SUPERVISOR_COMPLETE);
    }
}

static void h_mission_runtime_update_route(uint32_t now_ms,
    const line_sensor_snapshot_t *sensor,
    const wheel_odometry_snapshot_t *odometry)
{
    h_route_input_t route_input = {0};

    route_input.now_ms = now_ms;
    if (sensor != NULL) {
        route_input.line_active_count = sensor->active_count;
        route_input.line_sensor_ready = sensor->ready &&
            ((uint32_t) (now_ms - sensor->last_sample_ms) <=
                WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS);
    }
    if (odometry != NULL) {
        route_input.odometry_count = odometry->average_count;
        route_input.odometry_ready = odometry->ready;
    }
    HRouteEvents_Update(&g_route, &route_input);
}

static void h_mission_runtime_execute(uint32_t now_ms)
{
    motion_supervisor_snapshot_t motion = {0};

    (void) MotionSupervisor_GetSnapshot(&motion);
    switch (g_mission.snapshot.state) {
        case H_MISSION_STATE_RUNNING:
            if (!g_decel_active) {
                g_requested_base_speed_pps =
                    h_mission_runtime_speed_command(
                        g_mission.snapshot.elapsed_ms);
                if (g_route.snapshot.left_start_a &&
                    (g_route.snapshot.progress_count >=
                        H_MISSION_RUNTIME_DECEL_START_COUNT)) {
                    g_decel_active = true;
                    g_decel_started_ms = now_ms;
                    g_decel_start_speed_pps =
                        g_requested_base_speed_pps;
                }
            }
            if (g_decel_active) {
                g_requested_base_speed_pps =
                    h_mission_runtime_s_curve(
                        g_decel_start_speed_pps,
                        H_MISSION_RUNTIME_FINISH_SPEED_PPS,
                        now_ms - g_decel_started_ms,
                        H_MISSION_RUNTIME_DECEL_TIME_MS);
                g_speed_stage = H_MISSION_SPEED_STOPPING;
            } else {
                g_speed_stage =
                    (g_mission.snapshot.elapsed_ms <
                        H_MISSION_RUNTIME_ACCEL_TIME_MS) ?
                        H_MISSION_SPEED_RAMP :
                        H_MISSION_SPEED_CRUISE;
            }
            if (!LineFollowMission_SetBaseSpeed(
                    g_requested_base_speed_pps)) {
                g_executor_fault = true;
                g_snapshot.executor_error_count++;
                break;
            }
            if (!g_line_owned) {
                if (LineFollowMission_RequestStartFromWideMarker(
                        H_MISSION_RUNTIME_START_NARROW_ACTIVE_MAX)) {
                    g_line_owned = true;
                } else {
                    g_executor_fault = true;
                    g_snapshot.executor_error_count++;
                }
            }
            break;

        case H_MISSION_STATE_PRECISION_STOP:
            g_speed_stage = H_MISSION_SPEED_STOPPING;
            if (g_line_owned) {
                LineFollowMission_RequestStop();
                (void) LineFollowMission_SetBaseSpeed(
                    LINE_FOLLOW_MISSION_BASE_SPEED_PPS);
                if (!LineFollowMission_IsActive()) {
                    g_line_owned = false;
                }
                break;
            }
            if (!g_precision_owned &&
                BoardMotorSafe_IsHighImpedance() &&
                (g_route.config.precision_stop_delta_count != 0)) {
                if (MotionSupervisor_RequestRelativeHoldHeading(
                        g_route.config.precision_stop_delta_count,
                        H_MISSION_RUNTIME_PRECISION_SPEED_PPS,
                        H_MISSION_RUNTIME_PRECISION_TIMEOUT_MS)) {
                    g_precision_owned = true;
                } else {
                    g_executor_fault = true;
                    g_snapshot.executor_error_count++;
                }
            }
            break;

        case H_MISSION_STATE_FINISHED:
        case H_MISSION_STATE_FAULT:
        case H_MISSION_STATE_LOCKED:
            g_speed_stage = H_MISSION_SPEED_IDLE;
            g_requested_base_speed_pps = 0.0f;
            g_decel_active = false;
            h_mission_runtime_stop_owned();
            break;

        default:
            if (g_line_owned && !LineFollowMission_IsActive()) {
                g_line_owned = false;
            }
            if (g_precision_owned && !MotionSupervisor_IsActive() &&
                (motion.state != MOTION_SUPERVISOR_RUNNING)) {
                g_precision_owned = false;
            }
            break;
    }
}

static void h_mission_runtime_stop_owned(void)
{
    (void) LineFollowMission_SetBaseSpeed(
        LINE_FOLLOW_MISSION_BASE_SPEED_PPS);
    if (g_line_owned) {
        LineFollowMission_RequestStop();
        if (!LineFollowMission_IsActive()) {
            g_line_owned = false;
        }
    }
    if (g_precision_owned) {
        MotionSupervisor_RequestStop();
        if (!MotionSupervisor_IsActive()) {
            g_precision_owned = false;
        }
    }
}

static void h_mission_runtime_refresh_snapshot(void)
{
    (void) HMission_GetSnapshot(&g_mission, &g_snapshot.mission);
    (void) HRouteEvents_GetSnapshot(&g_route, &g_snapshot.route);
    g_snapshot.route_ready =
        g_snapshot.route.configuration_valid;
    g_snapshot.line_owned = g_line_owned;
    g_snapshot.precision_owned = g_precision_owned;
    g_snapshot.speed_stage = g_speed_stage;
    g_snapshot.requested_base_speed_pps =
        g_requested_base_speed_pps;
}

static float h_mission_runtime_speed_command(uint32_t elapsed_ms)
{
    return h_mission_runtime_s_curve(
        H_MISSION_RUNTIME_START_SPEED_PPS,
        H_MISSION_RUNTIME_CRUISE_SPEED_PPS,
        elapsed_ms, H_MISSION_RUNTIME_ACCEL_TIME_MS);
}

static float h_mission_runtime_s_curve(float start, float finish,
    uint32_t elapsed_ms, uint32_t duration_ms)
{
    float x;
    float x2;
    float x3;
    float smoother;

    if (elapsed_ms >= duration_ms) {
        return finish;
    }
    x = (float) elapsed_ms / (float) duration_ms;
    x2 = x * x;
    x3 = x2 * x;
    smoother = x3 * ((x * ((6.0f * x) - 15.0f)) + 10.0f);
    return start + ((finish - start) * smoother);
}
