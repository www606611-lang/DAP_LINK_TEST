#ifndef APP_MISSION_H_MISSION_H
#define APP_MISSION_H_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#define H_MISSION_BALL_STATUS_MAX_AGE_MS 250U
#define H_MISSION_H6_TARGET_LIMIT_0P1MM  1000
#define H_MISSION_H6_TARGET_STEP_0P1MM     50

typedef enum {
    H_MISSION_PROFILE_H2 = 2,
    H_MISSION_PROFILE_H3 = 3,
    H_MISSION_PROFILE_H4 = 4,
    H_MISSION_PROFILE_H5 = 5,
    H_MISSION_PROFILE_H6 = 6
} h_mission_profile_t;

typedef enum {
    H_MISSION_STATE_LOCKED = 0,
    H_MISSION_STATE_READY,
    H_MISSION_STATE_ARMED,
    H_MISSION_STATE_RUNNING,
    H_MISSION_STATE_PRECISION_STOP,
    H_MISSION_STATE_FINISHED,
    H_MISSION_STATE_FAULT
} h_mission_state_t;

typedef enum {
    H_MISSION_PHASE_IDLE = 0,
    H_MISSION_PHASE_BALL_ONLY,
    H_MISSION_PHASE_LEAVE_START_A,
    H_MISSION_PHASE_RUN_TO_B,
    H_MISSION_PHASE_RUN_TO_A,
    H_MISSION_PHASE_PRECISION_STOP
} h_mission_phase_t;

typedef enum {
    H_MISSION_FAULT_NONE = 0,
    H_MISSION_FAULT_OPERATOR_STOP,
    H_MISSION_FAULT_CHASSIS,
    H_MISSION_FAULT_LINE,
    H_MISSION_FAULT_K230_LINK,
    H_MISSION_FAULT_BALL_CONTROLLER
} h_mission_fault_t;

typedef enum {
    H_MISSION_CHASSIS_HOLD_HIGH_Z = 0,
    H_MISSION_CHASSIS_LINE_FOLLOW,
    H_MISSION_CHASSIS_PRECISION_STOP,
    H_MISSION_CHASSIS_STOP
} h_mission_chassis_action_t;

typedef enum {
    H_MISSION_BALL_IDLE = 0,
    H_MISSION_BALL_ARM,
    H_MISSION_BALL_RUN,
    H_MISSION_BALL_HOLD,
    H_MISSION_BALL_SAFE_STOP
} h_mission_ball_action_t;

typedef struct {
    uint32_t now_ms;
    bool unlock_requested;
    bool reset_requested;
    bool start_pressed;
    bool stop_requested;
    bool chassis_high_z;
    bool chassis_fault;
    bool line_ready;
    bool line_fault;
    bool left_start_a;
    bool b_passed;
    bool finish_a_passed;
    bool precision_stop_complete;
    bool k230_online;
    bool ball_ready;
    bool ball_armed;
    bool ball_finished;
    bool ball_fault;
    uint16_t ball_mission_sequence;
    uint16_t ball_status_age_ms;
} h_mission_input_t;

typedef struct {
    h_mission_state_t state;
    h_mission_phase_t phase;
    h_mission_profile_t profile;
    h_mission_fault_t fault;
    int16_t target_x_0p1mm;
    uint16_t mission_sequence;
    uint32_t run_count;
    uint32_t elapsed_ms;
    uint32_t b_passage_ms;
    uint32_t finish_ms;
} h_mission_snapshot_t;

typedef struct {
    h_mission_chassis_action_t chassis_action;
    h_mission_ball_action_t ball_action;
    h_mission_profile_t profile;
    int16_t target_x_0p1mm;
    uint16_t mission_sequence;
} h_mission_output_t;

typedef struct {
    h_mission_snapshot_t snapshot;
    int16_t h6_target_x_0p1mm;
    uint32_t started_ms;
} h_mission_t;

void HMission_Init(h_mission_t *mission, bool reset_locked);
bool HMission_SetProfile(h_mission_t *mission,
    h_mission_profile_t profile);
bool HMission_SetH6Target(h_mission_t *mission,
    int16_t target_x_0p1mm);
bool HMission_AdjustH6Target(h_mission_t *mission, int8_t steps);
void HMission_Step(h_mission_t *mission,
    const h_mission_input_t *input);
bool HMission_GetSnapshot(const h_mission_t *mission,
    h_mission_snapshot_t *snapshot);
bool HMission_GetOutput(const h_mission_t *mission,
    h_mission_output_t *output);
bool HMission_IsActive(const h_mission_t *mission);

#endif
