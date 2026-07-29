#include "wheel_line_tracking_control.h"

#include "icm20948.h"
#include "wheel_speed_control.h"

#include <assert.h>
#include <string.h>

static car_control_mode_t g_control_mode;
static wheel_speed_control_snapshot_t g_speed;
static float g_left_target_pps;
static float g_right_target_pps;

static void reset_mocks(void)
{
    memset(&g_speed, 0, sizeof(g_speed));
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_left_target_pps = 0.0f;
    g_right_target_pps = 0.0f;
}

static wheel_line_tracking_snapshot_t control_snapshot(void)
{
    wheel_line_tracking_snapshot_t snapshot;

    assert(WheelLineTrackingControl_GetSnapshot(&snapshot));
    return snapshot;
}

static void test_large_correction_uses_bounded_base_deceleration(void)
{
    wheel_line_tracking_snapshot_t snapshot;

    reset_mocks();
    WheelLineTrackingControl_Init(0U);
    assert(WheelLineTrackingControl_Start(
        3200.0f, 0, 2U, true, 0U, 0U) == WHEEL_LINE_TRACKING_OK);

    assert(WheelLineTrackingControl_SetCommand(
        3200.0f, 35, 2U, true, 10U, 10U) ==
        WHEEL_LINE_TRACKING_OK);
    WheelLineTrackingControl_Task(10U);
    snapshot = control_snapshot();
    assert(snapshot.base_speed_target_pps >= 3149.0f);
    assert(snapshot.base_speed_target_pps <= 3151.0f);
    assert(snapshot.correction_target_pps <= -899.0f);

    assert(WheelLineTrackingControl_SetCommand(
        3200.0f, 35, 2U, true, 20U, 20U) ==
        WHEEL_LINE_TRACKING_OK);
    WheelLineTrackingControl_Task(20U);
    snapshot = control_snapshot();
    assert(snapshot.base_speed_target_pps >= 3099.0f);
    assert(snapshot.base_speed_target_pps <= 3101.0f);
    assert(g_left_target_pps == snapshot.left_speed_target_pps);
    assert(g_right_target_pps == snapshot.right_speed_target_pps);
}

int main(void)
{
    test_large_correction_uses_bounded_base_deceleration();
    return 0;
}

car_control_mode_t ControlSupervisor_GetMode(void)
{
    return g_control_mode;
}

wheel_speed_control_result_t WheelSpeedControl_StartForMode(
    car_control_mode_t owner_mode, uint32_t now_ms)
{
    (void) now_ms;
    g_control_mode = owner_mode;
    g_speed.owner_mode = owner_mode;
    g_speed.running = true;
    return WHEEL_SPEED_CONTROL_OK;
}

wheel_speed_control_result_t WheelSpeedControl_SetTargets(
    float left_pps, float right_pps, uint32_t now_ms)
{
    (void) now_ms;
    g_left_target_pps = left_pps;
    g_right_target_pps = right_pps;
    return WHEEL_SPEED_CONTROL_OK;
}

void WheelSpeedControl_Stop(car_control_block_reason_t reason)
{
    (void) reason;
    g_speed.running = false;
    g_control_mode = CAR_CONTROL_MODE_SAFE_IDLE;
}

bool WheelSpeedControl_GetSnapshot(wheel_speed_control_snapshot_t *snapshot)
{
    *snapshot = g_speed;
    return true;
}

bool ICM20948_GetSnapshot(icm20948_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    return false;
}
