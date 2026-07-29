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
}

static h_route_config_t valid_route(void)
{
    const h_route_config_t config = {
        20, 80, 500, 900, 25, 6U, 20U, 20U
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

static void test_uncalibrated_route_cannot_arm_or_move(void)
{
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    HMissionRuntime_Init(false, 0U);
    task(0U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.profile == H_MISSION_PROFILE_H2);
    assert(snapshot.mission.state == H_MISSION_STATE_READY);
    assert(!snapshot.route_calibrated);

    HMissionRuntime_RequestPrimary();
    task(10U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_READY);
    assert(g_line_start_count == 0U);
    assert(g_motion_start_count == 0U);
}

static void test_reset_lock_cannot_arm_or_move(void)
{
    h_route_config_t config = valid_route();
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    HMissionRuntime_Init(true, 0U);
    assert(HMissionRuntime_SetRouteConfig(&config));
    task(0U);
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
    h_route_config_t config = valid_route();
    h_mission_runtime_snapshot_t snapshot;

    reset_mocks();
    HMissionRuntime_Init(false, 0U);
    assert(HMissionRuntime_SetRouteConfig(&config));
    task(10U);
    assert(runtime_snapshot().mission.state == H_MISSION_STATE_ARMED);

    HMissionRuntime_RequestPrimary();
    task(20U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_RUNNING);
    assert(snapshot.mission.phase == H_MISSION_PHASE_LEAVE_START_A);
    assert(snapshot.route.running);
    assert(g_line_start_count == 1U);

    g_sensor.active_count = 6U;
    task(30U);
    task(50U);
    assert(runtime_snapshot().route.initial_a_seen);

    g_sensor.active_count = 2U;
    g_odometry.average_count = 25;
    task(60U);
    task(80U);
    assert(runtime_snapshot().mission.phase == H_MISSION_PHASE_RUN_TO_B);

    g_odometry.average_count = 500;
    task(90U);
    assert(runtime_snapshot().mission.phase == H_MISSION_PHASE_RUN_TO_A);

    g_sensor.active_count = 6U;
    g_odometry.average_count = 900;
    task(100U);
    task(120U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_PRECISION_STOP);
    assert(g_line_stop_count >= 1U);

    task(130U);
    assert(g_motion_start_count == 1U);
    assert(g_motion_delta == config.precision_stop_delta_count);
    assert(runtime_snapshot().precision_owned);

    g_motion_active = false;
    g_motion.running = false;
    g_motion.state = MOTION_SUPERVISOR_COMPLETE;
    g_high_z = true;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    task(140U);
    snapshot = runtime_snapshot();
    assert(snapshot.mission.state == H_MISSION_STATE_FINISHED);
    assert(!snapshot.line_owned);
    assert(!snapshot.precision_owned);
}

static void test_stop_request_stops_owned_line(void)
{
    h_route_config_t config = valid_route();

    reset_mocks();
    HMissionRuntime_Init(false, 0U);
    assert(HMissionRuntime_SetRouteConfig(&config));
    task(0U);
    HMissionRuntime_RequestPrimary();
    task(10U);
    assert(HMissionRuntime_IsActive());

    HMissionRuntime_RequestStop();
    task(20U);
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
    test_uncalibrated_route_cannot_arm_or_move();
    test_reset_lock_cannot_arm_or_move();
    test_ordered_route_and_precision_executor();
    test_stop_request_stops_owned_line();
    return 0;
}
