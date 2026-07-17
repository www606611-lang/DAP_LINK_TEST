#ifndef APP_CAR_APP_H
#define APP_CAR_APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_APP_STATE_LOCKED = 0,
    CAR_APP_STATE_READY,
    CAR_APP_STATE_SERVICE,
    CAR_APP_STATE_MOTION_ACTIVE
} car_app_state_t;

typedef enum {
    CAR_APP_WORKFLOW_NONE = 0,
    CAR_APP_WORKFLOW_SPEED_TEST,
    CAR_APP_WORKFLOW_POSITION_TEST,
    CAR_APP_WORKFLOW_HEADING_TEST,
    CAR_APP_WORKFLOW_LINE_TEST,
    CAR_APP_WORKFLOW_YAW_TEST
} car_app_workflow_t;

typedef enum {
    CAR_APP_ACTION_NONE = 0,
    CAR_APP_ACTION_STOP_ACTIVE,
    CAR_APP_ACTION_START_YAW
} car_app_action_t;

typedef struct {
    bool service_active;
    bool speed_test_active;
    bool position_test_active;
    bool heading_test_active;
    bool line_test_active;
    bool yaw_test_active;
    bool pb21_press_event;
    bool pb4_press_event;
    bool pb5_press_event;
} car_app_inputs_t;

typedef struct {
    car_app_state_t state;
    car_app_workflow_t active_workflow;
    car_app_action_t action;
    int32_t yaw_command_mdeg;
    uint32_t transition_count;
} car_app_snapshot_t;

void CarApp_Init(bool reset_locked);
void CarApp_Step(const car_app_inputs_t *inputs);
bool CarApp_GetSnapshot(car_app_snapshot_t *snapshot);
const char *CarApp_GetStateText(void);

#endif
