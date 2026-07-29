#include "h_mission_runtime.h"

#include "board_motor_safe.h"
#include "control_supervisor.h"
#include "line_follow_mission.h"
#include "line_sensor.h"
#include "motion_supervisor.h"
#include "wheel_odometry.h"

#include <assert.h>
#include <string.h>

static bool g_high_z;
static car_control_mode_t g_control_mode;
static line_sensor_snapshot_t g_sensor;
static wheel_odometry_snapshot_t g_odometry;
static line_follow_mission_snapshot_t g_line;
static motion_supervisor_snapshot_t g_motion;
static bool g_line_active;
static bool g_motion_active;
static uint32_t g_line_start_count;
static uint32_t g_line_stop_count;
static uint32_t g_motion_start_count;
static int32_t g_motion_delta;
static bool g_line_started_from_marker;
static uint8_t g_line_start_narrow_max;
static float g_line_base_speed_pps;
static uint32_t g_line_base_set_count;

static void reset_mocks(void)
{
    memset(&g_sensor, 0, sizeof(g_sensor));
    memset(&g_odometry, 0, sizeof(g_odometry));
    memset(&g_line, 0, sizeof(g_line));
    memset(&g_motion, 0, sizeof(g_motion));
    g_high_z = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_sensor.ready = true;
    g_sensor.line_seen = true;
    g_sensor.active_count = 2U;
    g_odometry.ready = true;
    g_line.state = LINE_FOLLOW_MISSION_READY;
    g_motion.state = MOTION_SUPERVISOR_READY;
    g_line_active = false;
    g_motion_active = false;
    g_line_start_count = 0U;
    g_line_stop_count = 0U;
    g_motion_start_count = 0U;
    g_motion_delta = 0;
    g_line_started_from_marker = false;
    g_line_start_narrow_max = 0U;
    g_line_base_speed_pps = 1400.0f;
    g_line_base_set_count = 0U;
}

static h_route_config_t valid_route(void)
{
    const h_route_config_t config = {
        .precision_stop_delta_count = 0,
        .finish_arm_count = 500,
        .finish_rearm_ms = 300U,
        .marker_active_min = 5U,
        .finish_marker_active_min = 4U,
        .marker_confirm_ms = 20U,
        .finish_marker_confirm_ms = 10U,
        .marker_release_ms = 20U
    };
    return config;
}

static h_mission_runtime_snapshot_t runtime_snapshot(void)
{
    h_mission_runtime_snapshot_t snapshot;

    assert(HMissionRuntime_GetSnapshot(&snapshot));
    return snapshot;
}

static void task(uint32_t now_ms)
{
    g_sensor.last_sample_ms = now_ms;
    HMissionRuntime_Task(now_ms);
}

static void test_not_on_a_cannot_arm_or_move(void)
{
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    HMissionRuntime_Init(false, 0U);
    task(0U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.profile == H_MISSION_PROFILE_H2);
    assert(snapshot.mission.state == H_MISSION_STATE_READY);
    assert(snapshot.route_ready);
    assert(!snapshot.route.marker_wide);

    HMissionRuntime_RequestPrimary();
    task(10U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_READY);
    assert(g_line_start_count == 0U);
    assert(g_motion_start_count == 0U);
}

static void test_reset_lock_cannot_arm_or_move(void)
{
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    g_sensor.active_count = 5U;
    HMissionRuntime_Init(true, 0U);
    task(0U);
    task(20U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.profile == H_MISSION_PROFILE_H2);
    assert(snapshot.mission.state == H_MISSION_STATE_LOCKED);

    HMissionRuntime_RequestPrimary();
    task(10U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_LOCKED);
    assert(g_line_start_count == 0U);
    assert(g_motion_start_count == 0U);
}

static void test_ordered_route_and_precision_executor(void)
{
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    g_sensor.active_count = 5U;
    HMissionRuntime_Init(false, 0U);
    task(0U);
    task(20U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_ARMED);

    HMissionRuntime_RequestPrimary();
    task(30U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_RUNNING);
    assert(snapshot.mission.phase == H_MISSION_PHASE_LEAVE_START_A);
    assert(snapshot.route.running);
    assert(g_line_start_count == 1U);
    assert(g_line_started_from_marker);
    assert(g_line_start_narrow_max == 3U);

    task(31U);
    task(51U);
    assert(runtime_snapshot().route.initial_a_seen);

    g_sensor.active_count = 2U;
    g_odometry.average_count = 25;
    task(60U);
    task(80U);
    assert(runtime_snapshot().mission.phase == H_MISSION_PHASE_RUN_TO_A);

    g_odometry.average_count = 18999;
    task(379U);
    assert(!runtime_snapshot().route.finish_armed);
    g_odometry.average_count = 19000;
    task(380U);
    assert(runtime_snapshot().route.finish_armed);

    g_sensor.active_count = 4U;
    task(390U);
    task(400U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_PRECISION_STOP);
    assert(g_line_stop_count >= 1U);

    task(410U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_FINISHED);
    assert(!snapshot.line_owned);
    assert(!snapshot.precision_owned);
    assert(g_motion_start_count == 0U);
}

static void test_h2_s_curve_speed_profile(void)
{
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    g_sensor.active_count = 5U;
    HMissionRuntime_Init(false, 0U);
    task(0U);
    task(20U);
    HMissionRuntime_RequestPrimary();
    task(30U);
    snapshot = runtime_snapshot();
    assert(snapshot.speed_stage == H_MISSION_SPEED_RAMP);
    assert(g_line_base_speed_pps == 500.0f);

    task(1530U);
    snapshot = runtime_snapshot();
    assert(snapshot.speed_stage == H_MISSION_SPEED_RAMP);
    assert(g_line_base_speed_pps >= 1849.0f);
    assert(g_line_base_speed_pps <= 1851.0f);

    task(3030U);
    snapshot = runtime_snapshot();
    assert(snapshot.speed_stage == H_MISSION_SPEED_CRUISE);
    assert(g_line_base_speed_pps == 3200.0f);

    g_sensor.active_count = 2U;
    g_odometry.average_count = 25;
    task(3040U);
    task(3060U);
    g_odometry.average_count = 18000;
    task(3070U);
    snapshot = runtime_snapshot();
    assert(snapshot.speed_stage == H_MISSION_SPEED_STOPPING);
    assert(g_line_base_speed_pps == 3200.0f);

    task(4270U);
    assert(g_line_base_speed_pps >= 1949.0f);
    assert(g_line_base_speed_pps <= 1951.0f);
    task(5470U);
    assert(g_line_base_speed_pps == 700.0f);

    HMissionRuntime_RequestStop();
    task(5480U);
    assert(g_line_base_speed_pps == 1400.0f);
    assert(g_line_base_set_count >= 4U);
}

static void test_nonzero_precision_delta_uses_motion(void)
{
    h_route_config_t config = valid_route();

    config.precision_stop_delta_count = 25;
    reset_mocks();
    g_sensor.active_count = 5U;
    HMissionRuntime_Init(false, 0U);
    assert(HMissionRuntime_SetRouteConfig(&config));
    task(0U);
    task(20U);
    HMissionRuntime_RequestPrimary();
    task(30U);
    task(31U);
    task(51U);

    g_sensor.active_count = 2U;
    task(60U);
    task(80U);
    g_odometry.average_count = 500;
    task(380U);
    g_sensor.active_count = 4U;
    task(390U);
    task(400U);
    assert(runtime_snapshot().mission.state ==
        H_MISSION_STATE_PRECISION_STOP);

    task(410U);
    assert(g_motion_start_count == 1U);
    assert(g_motion_delta == config.precision_stop_delta_count);
    assert(runtime_snapshot().precision_owned);

    g_motion_active = false;
    g_motion.running = false;
    g_motion.state = MOTION_SUPERVISOR_COMPLETE;
    g_high_z = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    task(420U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_FINISHED);
}

static void test_stop_request_stops_owned_line(void)
{
    reset_mocks();
    g_sensor.active_count = 5U;
    HMissionRuntime_Init(false, 0U);
    task(0U);
    task(20U);
    HMissionRuntime_RequestPrimary();
    task(30U);
    assert(HMissionRuntime_IsActive());

    HMissionRuntime_RequestStop();
    task(40U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_FAULT);
    assert(runtime_snapshot().mission.fault ==
        H_MISSION_FAULT_OPERATOR_STOP);
    assert(g_line_stop_count >= 1U);
}

bool BoardMotorSafe_IsHighImpedance(void)
{
    return g_high_z;
}

car_control_mode_t ControlSupervisor_GetMode(void)
{
    return g_control_mode;
}

bool LineSensor_GetSnapshot(line_sensor_snapshot_t *snapshot)
{
    *snapshot = g_sensor;
    return true;
}

bool WheelOdometry_GetSnapshot(wheel_odometry_snapshot_t *snapshot)
{
    *snapshot = g_odometry;
    return true;
}

bool LineFollowMission_GetSnapshot(
    line_follow_mission_snapshot_t *snapshot)
{
    *snapshot = g_line;
    return true;
}

bool LineFollowMission_RequestStart(void)
{
    if (g_line_active || !g_high_z) {
        return false;
    }
    g_line_start_count++;
    g_line_active = true;
    g_line.state = LINE_FOLLOW_MISSION_RUNNING;
    g_high_z = false;
    g_control_mode = CAR_CONTROL_MODE_LINE_TRACKING;
    return true;
}

bool LineFollowMission_RequestStartFromWideMarker(
    uint8_t narrow_active_max)
{
    g_line_started_from_marker = true;
    g_line_start_narrow_max = narrow_active_max;
    return LineFollowMission_RequestStart();
}

bool LineFollowMission_SetBaseSpeed(float base_speed_pps)
{
    if ((base_speed_pps < 100.0f) ||
        (base_speed_pps > 5100.0f)) {
        return false;
    }
    g_line_base_speed_pps = base_speed_pps;
    g_line_base_set_count++;
    return true;
}

void LineFollowMission_RequestStop(void)
{
    g_line_stop_count++;
    g_line_active = false;
    g_line.state = LINE_FOLLOW_MISSION_STOPPED;
    g_high_z = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
}

bool LineFollowMission_IsActive(void)
{
    return g_line_active;
}

bool MotionSupervisor_GetSnapshot(
    motion_supervisor_snapshot_t *snapshot)
{
    *snapshot = g_motion;
    return true;
}

bool MotionSupervisor_RequestRelativeHoldHeading(
    int32_t delta_count, float max_speed_pps, uint32_t timeout_ms)
{
    (void) max_speed_pps;
    (void) timeout_ms;
    if (g_motion_active || !g_high_z) {
        return false;
    }
    g_motion_start_count++;
    g_motion_delta = delta_count;
    g_motion_active = true;
    g_motion.running = true;
    g_motion.state = MOTION_SUPERVISOR_RUNNING;
    g_high_z = false;
    g_control_mode = CAR_CONTROL_MODE_MOTION;
    return true;
}

void MotionSupervisor_RequestStop(void)
{
    g_motion_active = false;
    g_motion.running = false;
    g_motion.state = MOTION_SUPERVISOR_ABORTED;
    g_high_z = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
}

bool MotionSupervisor_IsActive(void)
{
    return g_motion_active;
}

int main(void)
{
    test_not_on_a_cannot_arm_or_move();
    test_reset_lock_cannot_arm_or_move();
    test_ordered_route_and_precision_executor();
    test_h2_s_curve_speed_profile();
    test_nonzero_precision_delta_uses_motion();
    test_stop_request_stops_owned_line();
    return 0;
}
