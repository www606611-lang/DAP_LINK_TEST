#include "h_mission.h"

#include <stddef.h>

#define H_MISSION_H3_FINAL_TARGET_0P1MM (-500)

static bool h_mission_profile_is_valid(h_mission_profile_t profile);
static bool h_mission_requires_line(h_mission_profile_t profile);
static bool h_mission_requires_ball(h_mission_profile_t profile);
static bool h_mission_ball_status_matches(const h_mission_t *mission,
    const h_mission_input_t *input);
static bool h_mission_can_arm(const h_mission_t *mission,
    const h_mission_input_t *input);
static int16_t h_mission_profile_target(const h_mission_t *mission,
    h_mission_profile_t profile);
static void h_mission_bump_sequence(h_mission_t *mission);
static void h_mission_prepare_ready(h_mission_t *mission);
static void h_mission_start(h_mission_t *mission, uint32_t now_ms);
static void h_mission_finish(h_mission_t *mission, uint32_t now_ms);
static void h_mission_fault(h_mission_t *mission,
    h_mission_fault_t fault, uint32_t now_ms);
static bool h_mission_active_inputs_are_safe(
    h_mission_t *mission, const h_mission_input_t *input);
static void h_mission_step_running(h_mission_t *mission,
    const h_mission_input_t *input);

void HMission_Init(h_mission_t *mission, bool reset_locked)
{
    if (mission == NULL) {
        return;
    }

    mission->snapshot.state = reset_locked ?
        H_MISSION_STATE_LOCKED : H_MISSION_STATE_READY;
    mission->snapshot.phase = H_MISSION_PHASE_IDLE;
    mission->snapshot.profile = H_MISSION_PROFILE_H2;
    mission->snapshot.fault = H_MISSION_FAULT_NONE;
    mission->snapshot.target_x_0p1mm = 0;
    mission->snapshot.mission_sequence = 1U;
    mission->snapshot.run_count = 0U;
    mission->snapshot.elapsed_ms = 0U;
    mission->snapshot.b_passage_ms = 0U;
    mission->snapshot.finish_ms = 0U;
    mission->h6_target_x_0p1mm = 0;
    mission->started_ms = 0U;
}

bool HMission_SetProfile(h_mission_t *mission,
    h_mission_profile_t profile)
{
    if ((mission == NULL) || !h_mission_profile_is_valid(profile) ||
        ((mission->snapshot.state != H_MISSION_STATE_READY) &&
         (mission->snapshot.state != H_MISSION_STATE_ARMED))) {
        return false;
    }
    if (mission->snapshot.profile == profile) {
        return true;
    }

    mission->snapshot.profile = profile;
    mission->snapshot.target_x_0p1mm =
        h_mission_profile_target(mission, profile);
    h_mission_bump_sequence(mission);
    h_mission_prepare_ready(mission);
    return true;
}

bool HMission_SetH6Target(h_mission_t *mission,
    int16_t target_x_0p1mm)
{
    if ((mission == NULL) ||
        (mission->snapshot.profile != H_MISSION_PROFILE_H6) ||
        ((mission->snapshot.state != H_MISSION_STATE_READY) &&
         (mission->snapshot.state != H_MISSION_STATE_ARMED)) ||
        (target_x_0p1mm < -H_MISSION_H6_TARGET_LIMIT_0P1MM) ||
        (target_x_0p1mm > H_MISSION_H6_TARGET_LIMIT_0P1MM)) {
        return false;
    }
    if (mission->h6_target_x_0p1mm == target_x_0p1mm) {
        return true;
    }

    mission->h6_target_x_0p1mm = target_x_0p1mm;
    mission->snapshot.target_x_0p1mm = target_x_0p1mm;
    h_mission_bump_sequence(mission);
    h_mission_prepare_ready(mission);
    return true;
}

bool HMission_AdjustH6Target(h_mission_t *mission, int8_t steps)
{
    int32_t target;

    if ((mission == NULL) || (steps == 0)) {
        return false;
    }
    target = (int32_t) mission->h6_target_x_0p1mm +
        ((int32_t) steps * H_MISSION_H6_TARGET_STEP_0P1MM);
    if (target < -H_MISSION_H6_TARGET_LIMIT_0P1MM) {
        target = -H_MISSION_H6_TARGET_LIMIT_0P1MM;
    } else if (target > H_MISSION_H6_TARGET_LIMIT_0P1MM) {
        target = H_MISSION_H6_TARGET_LIMIT_0P1MM;
    }
    return HMission_SetH6Target(mission, (int16_t) target);
}

void HMission_Step(h_mission_t *mission,
    const h_mission_input_t *input)
{
    if ((mission == NULL) || (input == NULL)) {
        return;
    }

    if (mission->snapshot.state == H_MISSION_STATE_LOCKED) {
        if (input->unlock_requested && input->chassis_high_z &&
            !input->chassis_fault) {
            h_mission_prepare_ready(mission);
        }
        return;
    }

    if ((mission->snapshot.state == H_MISSION_STATE_FINISHED) ||
        (mission->snapshot.state == H_MISSION_STATE_FAULT)) {
        if (input->reset_requested && input->chassis_high_z &&
            !input->chassis_fault) {
            h_mission_bump_sequence(mission);
            h_mission_prepare_ready(mission);
        }
        return;
    }

    if ((mission->snapshot.state == H_MISSION_STATE_RUNNING) ||
        (mission->snapshot.state == H_MISSION_STATE_PRECISION_STOP)) {
        if (!h_mission_active_inputs_are_safe(mission, input)) {
            return;
        }
        mission->snapshot.elapsed_ms =
            input->now_ms - mission->started_ms;
    }

    switch (mission->snapshot.state) {
        case H_MISSION_STATE_READY:
            if (h_mission_can_arm(mission, input)) {
                mission->snapshot.state = H_MISSION_STATE_ARMED;
                if (input->start_pressed) {
                    h_mission_start(mission, input->now_ms);
                }
            }
            break;

        case H_MISSION_STATE_ARMED:
            if (!h_mission_can_arm(mission, input)) {
                mission->snapshot.state = H_MISSION_STATE_READY;
            } else if (input->start_pressed) {
                h_mission_start(mission, input->now_ms);
            }
            break;

        case H_MISSION_STATE_RUNNING:
            h_mission_step_running(mission, input);
            break;

        case H_MISSION_STATE_PRECISION_STOP:
            if (input->precision_stop_complete) {
                h_mission_finish(mission, input->now_ms);
            }
            break;

        default:
            break;
    }
}

bool HMission_GetSnapshot(const h_mission_t *mission,
    h_mission_snapshot_t *snapshot)
{
    if ((mission == NULL) || (snapshot == NULL)) {
        return false;
    }
    *snapshot = mission->snapshot;
    return true;
}

bool HMission_GetOutput(const h_mission_t *mission,
    h_mission_output_t *output)
{
    bool requires_ball;

    if ((mission == NULL) || (output == NULL)) {
        return false;
    }

    requires_ball = h_mission_requires_ball(mission->snapshot.profile);
    output->profile = mission->snapshot.profile;
    output->target_x_0p1mm = mission->snapshot.target_x_0p1mm;
    output->mission_sequence = mission->snapshot.mission_sequence;
    output->chassis_action = H_MISSION_CHASSIS_HOLD_HIGH_Z;
    output->ball_action = H_MISSION_BALL_IDLE;

    switch (mission->snapshot.state) {
        case H_MISSION_STATE_READY:
        case H_MISSION_STATE_ARMED:
            output->ball_action = requires_ball ?
                H_MISSION_BALL_ARM : H_MISSION_BALL_IDLE;
            break;

        case H_MISSION_STATE_RUNNING:
            output->chassis_action =
                (mission->snapshot.profile == H_MISSION_PROFILE_H3) ?
                    H_MISSION_CHASSIS_HOLD_HIGH_Z :
                    H_MISSION_CHASSIS_LINE_FOLLOW;
            output->ball_action = requires_ball ?
                H_MISSION_BALL_RUN : H_MISSION_BALL_IDLE;
            break;

        case H_MISSION_STATE_PRECISION_STOP:
            output->chassis_action = H_MISSION_CHASSIS_PRECISION_STOP;
            break;

        case H_MISSION_STATE_FINISHED:
            output->chassis_action = H_MISSION_CHASSIS_STOP;
            output->ball_action = requires_ball ?
                H_MISSION_BALL_HOLD : H_MISSION_BALL_IDLE;
            break;

        case H_MISSION_STATE_FAULT:
            output->chassis_action = H_MISSION_CHASSIS_STOP;
            output->ball_action = requires_ball ?
                H_MISSION_BALL_SAFE_STOP : H_MISSION_BALL_IDLE;
            break;

        default:
            break;
    }
    return true;
}

bool HMission_IsActive(const h_mission_t *mission)
{
    return (mission != NULL) &&
        ((mission->snapshot.state == H_MISSION_STATE_RUNNING) ||
         (mission->snapshot.state == H_MISSION_STATE_PRECISION_STOP));
}

static bool h_mission_profile_is_valid(h_mission_profile_t profile)
{
    return (profile >= H_MISSION_PROFILE_H2) &&
        (profile <= H_MISSION_PROFILE_H6);
}

static bool h_mission_requires_line(h_mission_profile_t profile)
{
    return profile != H_MISSION_PROFILE_H3;
}

static bool h_mission_requires_ball(h_mission_profile_t profile)
{
    return profile >= H_MISSION_PROFILE_H3;
}

static bool h_mission_ball_status_matches(const h_mission_t *mission,
    const h_mission_input_t *input)
{
    return input->k230_online &&
        (input->ball_mission_sequence ==
            mission->snapshot.mission_sequence) &&
        (input->ball_status_age_ms <= H_MISSION_BALL_STATUS_MAX_AGE_MS);
}

static bool h_mission_can_arm(const h_mission_t *mission,
    const h_mission_input_t *input)
{
    if (!input->chassis_high_z || input->chassis_fault) {
        return false;
    }
    if (h_mission_requires_line(mission->snapshot.profile) &&
        (!input->line_ready || input->line_fault)) {
        return false;
    }
    if (h_mission_requires_ball(mission->snapshot.profile) &&
        (!h_mission_ball_status_matches(mission, input) ||
         !input->ball_ready || !input->ball_armed || input->ball_fault)) {
        return false;
    }
    return true;
}

static int16_t h_mission_profile_target(const h_mission_t *mission,
    h_mission_profile_t profile)
{
    if (profile == H_MISSION_PROFILE_H3) {
        return H_MISSION_H3_FINAL_TARGET_0P1MM;
    }
    if (profile == H_MISSION_PROFILE_H6) {
        return mission->h6_target_x_0p1mm;
    }
    return 0;
}

static void h_mission_bump_sequence(h_mission_t *mission)
{
    mission->snapshot.mission_sequence++;
    if (mission->snapshot.mission_sequence == 0U) {
        mission->snapshot.mission_sequence = 1U;
    }
}

static void h_mission_prepare_ready(h_mission_t *mission)
{
    mission->snapshot.state = H_MISSION_STATE_READY;
    mission->snapshot.phase = H_MISSION_PHASE_IDLE;
    mission->snapshot.fault = H_MISSION_FAULT_NONE;
    mission->snapshot.elapsed_ms = 0U;
    mission->snapshot.b_passage_ms = 0U;
    mission->snapshot.finish_ms = 0U;
    mission->started_ms = 0U;
}

static void h_mission_start(h_mission_t *mission, uint32_t now_ms)
{
    mission->started_ms = now_ms;
    mission->snapshot.elapsed_ms = 0U;
    mission->snapshot.b_passage_ms = 0U;
    mission->snapshot.finish_ms = 0U;
    mission->snapshot.fault = H_MISSION_FAULT_NONE;
    mission->snapshot.run_count++;
    mission->snapshot.state = H_MISSION_STATE_RUNNING;
    mission->snapshot.phase =
        (mission->snapshot.profile == H_MISSION_PROFILE_H3) ?
            H_MISSION_PHASE_BALL_ONLY :
            H_MISSION_PHASE_LEAVE_START_A;
}

static void h_mission_finish(h_mission_t *mission, uint32_t now_ms)
{
    mission->snapshot.elapsed_ms = now_ms - mission->started_ms;
    mission->snapshot.finish_ms = mission->snapshot.elapsed_ms;
    mission->snapshot.state = H_MISSION_STATE_FINISHED;
    mission->snapshot.fault = H_MISSION_FAULT_NONE;
}

static void h_mission_fault(h_mission_t *mission,
    h_mission_fault_t fault, uint32_t now_ms)
{
    mission->snapshot.elapsed_ms = now_ms - mission->started_ms;
    mission->snapshot.state = H_MISSION_STATE_FAULT;
    mission->snapshot.fault = fault;
}

static bool h_mission_active_inputs_are_safe(
    h_mission_t *mission, const h_mission_input_t *input)
{
    if (input->stop_requested || input->start_pressed) {
        h_mission_fault(mission,
            H_MISSION_FAULT_OPERATOR_STOP, input->now_ms);
        return false;
    }
    if (input->chassis_fault ||
        ((mission->snapshot.profile == H_MISSION_PROFILE_H3) &&
         !input->chassis_high_z)) {
        h_mission_fault(mission,
            H_MISSION_FAULT_CHASSIS, input->now_ms);
        return false;
    }
    if (h_mission_requires_line(mission->snapshot.profile) &&
        input->line_fault &&
        (mission->snapshot.state == H_MISSION_STATE_RUNNING)) {
        h_mission_fault(mission,
            H_MISSION_FAULT_LINE, input->now_ms);
        return false;
    }
    if (h_mission_requires_ball(mission->snapshot.profile)) {
        if (!h_mission_ball_status_matches(mission, input)) {
            h_mission_fault(mission,
                H_MISSION_FAULT_K230_LINK, input->now_ms);
            return false;
        }
        if (input->ball_fault) {
            h_mission_fault(mission,
                H_MISSION_FAULT_BALL_CONTROLLER, input->now_ms);
            return false;
        }
    }
    return true;
}

static void h_mission_step_running(h_mission_t *mission,
    const h_mission_input_t *input)
{
    switch (mission->snapshot.phase) {
        case H_MISSION_PHASE_BALL_ONLY:
            if (input->ball_finished) {
                h_mission_finish(mission, input->now_ms);
            }
            break;

        case H_MISSION_PHASE_LEAVE_START_A:
            if (input->left_start_a) {
                mission->snapshot.phase =
                    (mission->snapshot.profile == H_MISSION_PROFILE_H2) ?
                        H_MISSION_PHASE_RUN_TO_A :
                        H_MISSION_PHASE_RUN_TO_B;
            }
            break;

        case H_MISSION_PHASE_RUN_TO_B:
            if (input->b_passed) {
                mission->snapshot.b_passage_ms =
                    input->now_ms - mission->started_ms;
                if (mission->snapshot.profile == H_MISSION_PROFILE_H4) {
                    h_mission_finish(mission, input->now_ms);
                } else {
                    mission->snapshot.phase = H_MISSION_PHASE_RUN_TO_A;
                }
            }
            break;

        case H_MISSION_PHASE_RUN_TO_A:
            if (input->finish_a_passed) {
                if (mission->snapshot.profile == H_MISSION_PROFILE_H2) {
                    mission->snapshot.phase =
                        H_MISSION_PHASE_PRECISION_STOP;
                    mission->snapshot.state =
                        H_MISSION_STATE_PRECISION_STOP;
                } else {
                    h_mission_finish(mission, input->now_ms);
                }
            }
            break;

        default:
            break;
    }
}
