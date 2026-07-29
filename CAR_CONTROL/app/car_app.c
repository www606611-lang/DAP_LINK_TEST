#include "car_app.h"

#include <stddef.h>

static bool g_reset_locked;
static car_app_snapshot_t g_snapshot;

static car_app_workflow_t car_app_get_active_workflow(
    const car_app_inputs_t *inputs);
static void car_app_set_state(car_app_state_t state);

void CarApp_Init(bool reset_locked)
{
    g_reset_locked = reset_locked;
    g_snapshot.state = reset_locked ?
        CAR_APP_STATE_LOCKED : CAR_APP_STATE_READY;
    g_snapshot.active_workflow = CAR_APP_WORKFLOW_NONE;
    g_snapshot.action = CAR_APP_ACTION_NONE;
    g_snapshot.transition_count = 0U;
}

void CarApp_Step(const car_app_inputs_t *inputs)
{
    bool any_press_event;

    if (inputs == NULL) {
        return;
    }

    g_snapshot.action = CAR_APP_ACTION_NONE;
    g_snapshot.active_workflow =
        car_app_get_active_workflow(inputs);
    any_press_event = inputs->pb21_press_event ||
        inputs->pb4_press_event || inputs->pb5_press_event;

    if (g_snapshot.active_workflow != CAR_APP_WORKFLOW_NONE) {
        car_app_set_state(CAR_APP_STATE_MOTION_ACTIVE);
        if (any_press_event) {
            g_snapshot.action = CAR_APP_ACTION_STOP_ACTIVE;
        }
        return;
    }

    if (g_reset_locked) {
        car_app_set_state(CAR_APP_STATE_LOCKED);
        return;
    }
    if (inputs->service_active) {
        car_app_set_state(CAR_APP_STATE_SERVICE);
        return;
    }

    car_app_set_state(CAR_APP_STATE_READY);
    if (inputs->pb21_press_event) {
        g_snapshot.action = CAR_APP_ACTION_H_PRIMARY;
    }
}

bool CarApp_GetSnapshot(car_app_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

const char *CarApp_GetStateText(void)
{
    switch (g_snapshot.state) {
        case CAR_APP_STATE_LOCKED:
            return "LOCKED";
        case CAR_APP_STATE_READY:
            return "READY";
        case CAR_APP_STATE_SERVICE:
            return "SERVICE";
        case CAR_APP_STATE_MOTION_ACTIVE:
            return "ACTIVE";
        default:
            return "UNKNOWN";
    }
}

static car_app_workflow_t car_app_get_active_workflow(
    const car_app_inputs_t *inputs)
{
    if (inputs->speed_test_active) {
        return CAR_APP_WORKFLOW_SPEED_TEST;
    }
    if (inputs->position_test_active) {
        return CAR_APP_WORKFLOW_POSITION_TEST;
    }
    if (inputs->heading_test_active) {
        return CAR_APP_WORKFLOW_HEADING_TEST;
    }
    if (inputs->line_test_active) {
        return CAR_APP_WORKFLOW_LINE_TEST;
    }
    if (inputs->h_mission_active) {
        return CAR_APP_WORKFLOW_H_MISSION;
    }
    if (inputs->line_mission_active) {
        return CAR_APP_WORKFLOW_LINE_MISSION;
    }
    if (inputs->motion_active) {
        return CAR_APP_WORKFLOW_MOTION;
    }
    if (inputs->yaw_test_active) {
        return CAR_APP_WORKFLOW_YAW_TEST;
    }
    return CAR_APP_WORKFLOW_NONE;
}

static void car_app_set_state(car_app_state_t state)
{
    if (g_snapshot.state != state) {
        g_snapshot.state = state;
        g_snapshot.transition_count++;
    }
}
