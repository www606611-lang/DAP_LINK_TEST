#include "car_app.h"

#include <assert.h>
#include <string.h>

static car_app_snapshot_t step(const car_app_inputs_t *inputs)
{
    car_app_snapshot_t snapshot;

    CarApp_Step(inputs);
    assert(CarApp_GetSnapshot(&snapshot));
    return snapshot;
}

static void test_ready_button_commands(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);

    inputs.pb21_press_event = true;
    snapshot = step(&inputs);
    assert(snapshot.state == CAR_APP_STATE_READY);
    assert(snapshot.action == CAR_APP_ACTION_H_PRIMARY);

    inputs.pb21_press_event = false;
    inputs.pb4_press_event = true;
    snapshot = step(&inputs);
    assert(snapshot.action == CAR_APP_ACTION_NONE);

    inputs.pb4_press_event = false;
    inputs.pb5_press_event = true;
    snapshot = step(&inputs);
    assert(snapshot.action == CAR_APP_ACTION_NONE);
}

static void test_button_stops_h_mission_before_child_workflow(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.h_mission_active = true;
    inputs.line_mission_active = true;
    inputs.pb4_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.state == CAR_APP_STATE_MOTION_ACTIVE);
    assert(snapshot.active_workflow == CAR_APP_WORKFLOW_H_MISSION);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_motion_button_stops_active_workflow(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.line_test_active = true;
    inputs.pb21_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.state == CAR_APP_STATE_MOTION_ACTIVE);
    assert(snapshot.active_workflow == CAR_APP_WORKFLOW_LINE_TEST);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_button_stops_formal_line_mission(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.line_mission_active = true;
    inputs.pb21_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.state == CAR_APP_STATE_MOTION_ACTIVE);
    assert(snapshot.active_workflow ==
        CAR_APP_WORKFLOW_LINE_MISSION);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_button_stops_motion_workflow(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.motion_active = true;
    inputs.pb21_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.state == CAR_APP_STATE_MOTION_ACTIVE);
    assert(snapshot.active_workflow == CAR_APP_WORKFLOW_MOTION);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_workflow_priority_is_deterministic(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.speed_test_active = true;
    inputs.yaw_test_active = true;
    inputs.pb5_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.active_workflow == CAR_APP_WORKFLOW_SPEED_TEST);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_service_and_lockout_block_new_motion(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    inputs.service_active = true;
    inputs.pb21_press_event = true;
    CarApp_Init(false);
    snapshot = step(&inputs);
    assert(snapshot.state == CAR_APP_STATE_SERVICE);
    assert(snapshot.action == CAR_APP_ACTION_NONE);

    CarApp_Init(true);
    inputs.service_active = false;
    snapshot = step(&inputs);
    assert(snapshot.state == CAR_APP_STATE_LOCKED);
    assert(snapshot.action == CAR_APP_ACTION_NONE);
}

static void test_button_still_stops_motion_during_service(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    inputs.service_active = true;
    inputs.heading_test_active = true;
    inputs.pb4_press_event = true;
    snapshot = step(&inputs);

    assert(snapshot.state == CAR_APP_STATE_MOTION_ACTIVE);
    assert(snapshot.active_workflow == CAR_APP_WORKFLOW_HEADING_TEST);
    assert(snapshot.action == CAR_APP_ACTION_STOP_ACTIVE);
}

static void test_transition_count_tracks_state_changes(void)
{
    car_app_inputs_t inputs;
    car_app_snapshot_t snapshot;

    memset(&inputs, 0, sizeof(inputs));
    CarApp_Init(false);
    snapshot = step(&inputs);
    assert(snapshot.transition_count == 0U);

    inputs.service_active = true;
    snapshot = step(&inputs);
    assert(snapshot.transition_count == 1U);

    snapshot = step(&inputs);
    assert(snapshot.transition_count == 1U);

    inputs.service_active = false;
    snapshot = step(&inputs);
    assert(snapshot.transition_count == 2U);
}

int main(void)
{
    test_ready_button_commands();
    test_button_stops_h_mission_before_child_workflow();
    test_motion_button_stops_active_workflow();
    test_button_stops_formal_line_mission();
    test_button_stops_motion_workflow();
    test_workflow_priority_is_deterministic();
    test_service_and_lockout_block_new_motion();
    test_button_still_stops_motion_during_service();
    test_transition_count_tracks_state_changes();
    return 0;
}
