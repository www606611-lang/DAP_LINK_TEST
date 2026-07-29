#include "h_mission.h"

#include <assert.h>
#include <string.h>

static h_mission_input_t ready_input(const h_mission_t *mission,
    uint32_t now_ms)
{
    h_mission_input_t input;

    memset(&input, 0, sizeof(input));
    input.now_ms = now_ms;
    input.chassis_high_z = true;
    input.line_ready = true;
    input.k230_online = true;
    input.ball_ready = true;
    input.ball_armed = true;
    input.ball_mission_sequence = mission->snapshot.mission_sequence;
    input.ball_status_age_ms = 10U;
    return input;
}

static void arm_and_start(h_mission_t *mission,
    h_mission_input_t *input, uint32_t now_ms)
{
    *input = ready_input(mission, now_ms);
    HMission_Step(mission, input);
    assert(mission->snapshot.state == H_MISSION_STATE_ARMED);
    input->now_ms = now_ms + 1U;
    input->start_pressed = true;
    HMission_Step(mission, input);
    input->start_pressed = false;
    assert(mission->snapshot.state == H_MISSION_STATE_RUNNING);
}

static void test_locked_requires_safe_unlock(void)
{
    h_mission_t mission;
    h_mission_input_t input;

    HMission_Init(&mission, true);
    input = ready_input(&mission, 10U);
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_LOCKED);

    input.unlock_requested = true;
    input.chassis_high_z = false;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_LOCKED);

    input.chassis_high_z = true;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_READY);
}

static void test_h2_ignores_initial_a_and_precision_stops(void)
{
    h_mission_t mission;
    h_mission_input_t input;
    h_mission_output_t output;

    HMission_Init(&mission, false);
    arm_and_start(&mission, &input, 100U);
    assert(mission.snapshot.phase == H_MISSION_PHASE_LEAVE_START_A);

    input.now_ms = 150U;
    input.finish_a_passed = true;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.phase == H_MISSION_PHASE_LEAVE_START_A);

    input.finish_a_passed = false;
    input.left_start_a = true;
    input.now_ms = 200U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.phase == H_MISSION_PHASE_RUN_TO_B);

    input.left_start_a = false;
    input.finish_a_passed = true;
    input.now_ms = 250U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.phase == H_MISSION_PHASE_RUN_TO_B);

    input.finish_a_passed = false;
    input.b_passed = true;
    input.now_ms = 500U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.phase == H_MISSION_PHASE_RUN_TO_A);
    assert(mission.snapshot.b_passage_ms == 399U);

    input.b_passed = false;
    input.finish_a_passed = true;
    input.now_ms = 900U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_PRECISION_STOP);
    assert(HMission_GetOutput(&mission, &output));
    assert(output.chassis_action == H_MISSION_CHASSIS_PRECISION_STOP);

    input.finish_a_passed = false;
    input.precision_stop_complete = true;
    input.now_ms = 950U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FINISHED);
    assert(mission.snapshot.finish_ms == 849U);
    assert(HMission_GetOutput(&mission, &output));
    assert(output.chassis_action == H_MISSION_CHASSIS_STOP);
    assert(output.ball_action == H_MISSION_BALL_IDLE);
}

static void test_h3_is_ball_only_and_holds_final_target(void)
{
    h_mission_t mission;
    h_mission_input_t input;
    h_mission_output_t output;

    HMission_Init(&mission, false);
    assert(HMission_SetProfile(&mission, H_MISSION_PROFILE_H3));
    assert(mission.snapshot.target_x_0p1mm == -500);
    arm_and_start(&mission, &input, 1000U);
    assert(mission.snapshot.phase == H_MISSION_PHASE_BALL_ONLY);
    assert(HMission_GetOutput(&mission, &output));
    assert(output.chassis_action == H_MISSION_CHASSIS_HOLD_HIGH_Z);
    assert(output.ball_action == H_MISSION_BALL_RUN);

    input.now_ms = 4100U;
    input.ball_finished = true;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FINISHED);
    assert(mission.snapshot.finish_ms == 3099U);
    assert(!HMission_SetProfile(&mission, H_MISSION_PROFILE_H2));
    assert(HMission_GetOutput(&mission, &output));
    assert(output.ball_action == H_MISSION_BALL_HOLD);
}

static void test_h4_finishes_at_b(void)
{
    h_mission_t mission;
    h_mission_input_t input;

    HMission_Init(&mission, false);
    assert(HMission_SetProfile(&mission, H_MISSION_PROFILE_H4));
    arm_and_start(&mission, &input, 10U);
    input.start_pressed = false;
    input.chassis_high_z = false;
    input.left_start_a = true;
    input.now_ms = 20U;
    HMission_Step(&mission, &input);
    input.left_start_a = false;
    input.b_passed = true;
    input.now_ms = 7000U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FINISHED);
    assert(mission.snapshot.b_passage_ms == 6989U);
    assert(mission.snapshot.finish_ms == 6989U);
}

static void test_h5_requires_b_before_finish_a(void)
{
    h_mission_t mission;
    h_mission_input_t input;

    HMission_Init(&mission, false);
    assert(HMission_SetProfile(&mission, H_MISSION_PROFILE_H5));
    arm_and_start(&mission, &input, 100U);
    input.chassis_high_z = false;
    input.left_start_a = true;
    input.finish_a_passed = true;
    input.now_ms = 200U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.phase == H_MISSION_PHASE_RUN_TO_B);

    input.left_start_a = false;
    input.finish_a_passed = false;
    input.b_passed = true;
    input.now_ms = 500U;
    HMission_Step(&mission, &input);
    input.b_passed = false;
    input.finish_a_passed = true;
    input.now_ms = 900U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FINISHED);
}

static void test_h6_target_and_sequence_rules(void)
{
    h_mission_t mission;
    h_mission_input_t input;
    uint16_t sequence;

    HMission_Init(&mission, false);
    assert(HMission_SetProfile(&mission, H_MISSION_PROFILE_H6));
    sequence = mission.snapshot.mission_sequence;
    assert(HMission_AdjustH6Target(&mission, 2));
    assert(mission.snapshot.target_x_0p1mm == 100);
    assert(mission.snapshot.mission_sequence != sequence);
    assert(HMission_AdjustH6Target(&mission, 127));
    assert(mission.snapshot.target_x_0p1mm == 1000);
    assert(!HMission_SetH6Target(&mission, 1001));

    arm_and_start(&mission, &input, 100U);
    assert(!HMission_AdjustH6Target(&mission, -1));
    assert(!HMission_SetProfile(&mission, H_MISSION_PROFILE_H2));
}

static void test_active_faults_are_latched_and_reset_is_new_sequence(void)
{
    h_mission_t mission;
    h_mission_input_t input;
    h_mission_output_t output;
    uint16_t sequence;

    HMission_Init(&mission, false);
    assert(HMission_SetProfile(&mission, H_MISSION_PROFILE_H5));
    arm_and_start(&mission, &input, 100U);
    input.chassis_high_z = false;
    input.now_ms = 200U;
    input.ball_status_age_ms = H_MISSION_BALL_STATUS_MAX_AGE_MS + 1U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FAULT);
    assert(mission.snapshot.fault == H_MISSION_FAULT_K230_LINK);
    assert(HMission_GetOutput(&mission, &output));
    assert(output.chassis_action == H_MISSION_CHASSIS_STOP);
    assert(output.ball_action == H_MISSION_BALL_SAFE_STOP);

    sequence = mission.snapshot.mission_sequence;
    input = ready_input(&mission, 300U);
    input.reset_requested = true;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_READY);
    assert(mission.snapshot.mission_sequence != sequence);
}

static void test_any_active_button_stops_immediately(void)
{
    h_mission_t mission;
    h_mission_input_t input;

    HMission_Init(&mission, false);
    arm_and_start(&mission, &input, 10U);
    input.start_pressed = true;
    input.now_ms = 20U;
    HMission_Step(&mission, &input);
    assert(mission.snapshot.state == H_MISSION_STATE_FAULT);
    assert(mission.snapshot.fault == H_MISSION_FAULT_OPERATOR_STOP);
}

int main(void)
{
    test_locked_requires_safe_unlock();
    test_h2_ignores_initial_a_and_precision_stops();
    test_h3_is_ball_only_and_holds_final_target();
    test_h4_finishes_at_b();
    test_h5_requires_b_before_finish_a();
    test_h6_target_and_sequence_rules();
    test_active_faults_are_latched_and_reset_is_new_sequence();
    test_any_active_button_stops_immediately();
    return 0;
}
