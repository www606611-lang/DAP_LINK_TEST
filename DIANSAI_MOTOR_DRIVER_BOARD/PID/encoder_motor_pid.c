#include "encoder_motor_pid.h"

#include <stddef.h>

static int encoder_motor_pid_round_to_int(float value);
static float encoder_motor_pid_abs(float value);
static float encoder_motor_pid_clamp(
    float value, float min_value, float max_value);
static float encoder_motor_pid_apply_feedforward(
    float pid_output, float speed_target, float feedforward_pwm,
    float reference_pps);
static float encoder_motor_pid_apply_min_drive(
    float pwm_command, float speed_target, float speed_measurement,
    float forward_min_drive_pwm, float reverse_min_drive_pwm,
    float reference_pps);
static float encoder_motor_pid_limit_to_target_direction(
    float pwm_command, float speed_target);
static void encoder_motor_pid_apply_pwm(
    motor_id_t motor_id, int pwm_command);

void EncoderMotorPID_Init(encoder_motor_pid_t *controller,
    motor_id_t motor_id, encoder_id_t encoder_id, uint32_t now_ms)
{
    if (controller == NULL) {
        return;
    }

    controller->motor_id = motor_id;
    controller->encoder_id = encoder_id;
    PID_Init(&controller->speed_pid);
    PID_Init(&controller->position_pid);
    controller->snapshot.count = 0;
    controller->snapshot.delta = 0;
    controller->snapshot.speed_pps = 0;
    controller->snapshot.direction = 0;
    controller->target_speed_pps = 0.0f;
    controller->target_position_count = 0.0f;
    controller->cascade_speed_target_pps = 0.0f;
    controller->pwm_command = 0.0f;
    controller->speed_feedforward_pwm = 0.0f;
    controller->speed_feedforward_reference_pps = 0.0f;
    controller->speed_forward_min_drive_pwm = 0.0f;
    controller->speed_reverse_min_drive_pwm = 0.0f;
    controller->speed_min_drive_reference_pps = 0.0f;
    controller->last_update_ms = now_ms;
    controller->mode = ENCODER_MOTOR_PID_MODE_STOP;
    controller->enabled = false;

    EncoderMotorPID_SetSpeedOutputLimits(controller, -(float) MOTOR_PWM_MAX,
        (float) MOTOR_PWM_MAX);
}

void EncoderMotorPID_Reset(encoder_motor_pid_t *controller, uint32_t now_ms)
{
    if (controller == NULL) {
        return;
    }

    PID_Reset(&controller->speed_pid);
    PID_Reset(&controller->position_pid);
    controller->snapshot.count = 0;
    controller->snapshot.delta = 0;
    controller->snapshot.speed_pps = 0;
    controller->snapshot.direction = 0;
    controller->cascade_speed_target_pps = 0.0f;
    controller->pwm_command = 0.0f;
    controller->last_update_ms = now_ms;
}

void EncoderMotorPID_Stop(encoder_motor_pid_t *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->enabled = false;
    controller->mode = ENCODER_MOTOR_PID_MODE_STOP;
    controller->target_speed_pps = 0.0f;
    controller->cascade_speed_target_pps = 0.0f;
    controller->pwm_command = 0.0f;
    PID_Reset(&controller->speed_pid);
    PID_Reset(&controller->position_pid);
    encoder_motor_pid_apply_pwm(controller->motor_id, 0);
}

void EncoderMotorPID_SetSpeedTunings(encoder_motor_pid_t *controller,
    float kp, float ki, float kd)
{
    if (controller == NULL) {
        return;
    }

    PID_SetTunings(&controller->speed_pid, kp, ki, kd);
}

void EncoderMotorPID_SetPositionTunings(encoder_motor_pid_t *controller,
    float kp, float ki, float kd)
{
    if (controller == NULL) {
        return;
    }

    PID_SetTunings(&controller->position_pid, kp, ki, kd);
}

void EncoderMotorPID_SetSpeedOutputLimits(encoder_motor_pid_t *controller,
    float output_min, float output_max)
{
    if (controller == NULL) {
        return;
    }

    PID_SetOutputLimits(&controller->speed_pid, output_min, output_max);
}

void EncoderMotorPID_SetPositionOutputLimits(encoder_motor_pid_t *controller,
    float output_min, float output_max)
{
    if (controller == NULL) {
        return;
    }

    PID_SetOutputLimits(&controller->position_pid, output_min, output_max);
}

void EncoderMotorPID_SetSpeedIntegralLimits(encoder_motor_pid_t *controller,
    float integral_min, float integral_max)
{
    if (controller == NULL) {
        return;
    }

    PID_SetIntegralLimits(&controller->speed_pid, integral_min, integral_max);
}

void EncoderMotorPID_SetPositionIntegralLimits(encoder_motor_pid_t *controller,
    float integral_min, float integral_max)
{
    if (controller == NULL) {
        return;
    }

    PID_SetIntegralLimits(&controller->position_pid, integral_min,
        integral_max);
}

void EncoderMotorPID_SetSpeedFeedforwardPwm(encoder_motor_pid_t *controller,
    float feedforward_pwm)
{
    if (controller == NULL) {
        return;
    }

    controller->speed_feedforward_pwm = encoder_motor_pid_abs(feedforward_pwm);
}

void EncoderMotorPID_SetSpeedFeedforwardReferencePps(
    encoder_motor_pid_t *controller, float reference_pps)
{
    if (controller == NULL) {
        return;
    }

    controller->speed_feedforward_reference_pps =
        encoder_motor_pid_abs(reference_pps);
}

void EncoderMotorPID_SetSpeedMinDrivePwm(encoder_motor_pid_t *controller,
    float min_drive_pwm)
{
    if (controller == NULL) {
        return;
    }

    EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
        controller, min_drive_pwm, min_drive_pwm);
}

void EncoderMotorPID_SetSpeedDirectionalMinDrivePwm(
    encoder_motor_pid_t *controller, float forward_pwm, float reverse_pwm)
{
    if (controller == NULL) {
        return;
    }

    controller->speed_forward_min_drive_pwm =
        encoder_motor_pid_abs(forward_pwm);
    controller->speed_reverse_min_drive_pwm =
        encoder_motor_pid_abs(reverse_pwm);
}

void EncoderMotorPID_SetSpeedMinDriveReferencePps(
    encoder_motor_pid_t *controller, float reference_pps)
{
    if (controller == NULL) {
        return;
    }

    controller->speed_min_drive_reference_pps =
        encoder_motor_pid_abs(reference_pps);
}

void EncoderMotorPID_SetSpeedDeadband(encoder_motor_pid_t *controller,
    float deadband)
{
    if (controller == NULL) {
        return;
    }

    PID_SetDeadband(&controller->speed_pid, deadband);
}

void EncoderMotorPID_SetPositionDeadband(encoder_motor_pid_t *controller,
    float deadband)
{
    if (controller == NULL) {
        return;
    }

    PID_SetDeadband(&controller->position_pid, deadband);
}

void EncoderMotorPID_SetTargetSpeedPps(encoder_motor_pid_t *controller,
    float target_speed_pps)
{
    if (controller == NULL) {
        return;
    }

    controller->target_speed_pps = target_speed_pps;
    controller->cascade_speed_target_pps = target_speed_pps;
    controller->mode = ENCODER_MOTOR_PID_MODE_SPEED;
    controller->enabled = true;
}

void EncoderMotorPID_SetTargetPositionCount(encoder_motor_pid_t *controller,
    float target_position_count)
{
    if (controller == NULL) {
        return;
    }

    controller->target_position_count = target_position_count;
    controller->mode = ENCODER_MOTOR_PID_MODE_POSITION;
    controller->enabled = true;
}

bool EncoderMotorPID_Update(encoder_motor_pid_t *controller, uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float dt_s;
    float speed_target;

    if (controller == NULL) {
        return false;
    }
    if (!controller->enabled) {
        return false;
    }

    elapsed_ms = now_ms - controller->last_update_ms;
    if (elapsed_ms < ENCODER_SAMPLE_INTERVAL_MS) {
        return false;
    }

    if (!Encoder_GetSnapshot(controller->encoder_id, &controller->snapshot)) {
        return false;
    }

    dt_s = (float) elapsed_ms / 1000.0f;
    controller->last_update_ms = now_ms;

    if (controller->mode == ENCODER_MOTOR_PID_MODE_POSITION) {
        controller->cascade_speed_target_pps = PID_Update(
            &controller->position_pid, controller->target_position_count,
            (float) controller->snapshot.count, dt_s);
        speed_target = controller->cascade_speed_target_pps;
    } else {
        controller->cascade_speed_target_pps = controller->target_speed_pps;
        speed_target = controller->target_speed_pps;
    }

    controller->pwm_command = PID_Update(
        &controller->speed_pid, speed_target,
        (float) controller->snapshot.speed_pps, dt_s);
    controller->pwm_command = encoder_motor_pid_apply_feedforward(
        controller->pwm_command, speed_target,
        controller->speed_feedforward_pwm,
        controller->speed_feedforward_reference_pps);
    controller->pwm_command = encoder_motor_pid_apply_min_drive(
        controller->pwm_command, speed_target,
        (float) controller->snapshot.speed_pps,
        controller->speed_forward_min_drive_pwm,
        controller->speed_reverse_min_drive_pwm,
        controller->speed_min_drive_reference_pps);
    controller->pwm_command = encoder_motor_pid_clamp(
        controller->pwm_command, controller->speed_pid.output_min,
        controller->speed_pid.output_max);
    controller->pwm_command = encoder_motor_pid_limit_to_target_direction(
        controller->pwm_command, speed_target);

    encoder_motor_pid_apply_pwm(controller->motor_id,
        encoder_motor_pid_round_to_int(controller->pwm_command));
    return true;
}

float EncoderMotorPID_GetPwmCommand(
    const encoder_motor_pid_t *controller)
{
    if (controller == NULL) {
        return 0.0f;
    }

    return controller->pwm_command;
}

float EncoderMotorPID_GetSpeedTargetPps(
    const encoder_motor_pid_t *controller)
{
    if (controller == NULL) {
        return 0.0f;
    }

    if (controller->mode == ENCODER_MOTOR_PID_MODE_SPEED) {
        return controller->target_speed_pps;
    }

    return controller->cascade_speed_target_pps;
}

float EncoderMotorPID_GetTargetPositionCount(
    const encoder_motor_pid_t *controller)
{
    if (controller == NULL) {
        return 0.0f;
    }

    return controller->target_position_count;
}

encoder_motor_pid_mode_t EncoderMotorPID_GetMode(
    const encoder_motor_pid_t *controller)
{
    if (controller == NULL) {
        return ENCODER_MOTOR_PID_MODE_STOP;
    }

    return controller->mode;
}

bool EncoderMotorPID_GetSnapshot(const encoder_motor_pid_t *controller,
    encoder_snapshot_t *snapshot)
{
    if ((controller == NULL) || (snapshot == NULL)) {
        return false;
    }

    *snapshot = controller->snapshot;
    return true;
}

static int encoder_motor_pid_round_to_int(float value)
{
    if (value >= 0.0f) {
        return (int) (value + 0.5f);
    }

    return (int) (value - 0.5f);
}

static float encoder_motor_pid_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float encoder_motor_pid_clamp(
    float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float encoder_motor_pid_apply_feedforward(
    float pid_output, float speed_target, float feedforward_pwm,
    float reference_pps)
{
    float scaled_feedforward;

    if ((feedforward_pwm <= 0.0f) || (speed_target == 0.0f)) {
        return pid_output;
    }

    scaled_feedforward = feedforward_pwm;
    if (reference_pps > 0.0f) {
        scaled_feedforward = feedforward_pwm *
            (encoder_motor_pid_abs(speed_target) / reference_pps);
        if (scaled_feedforward > feedforward_pwm) {
            scaled_feedforward = feedforward_pwm;
        }
    }

    return (speed_target > 0.0f) ? (pid_output + scaled_feedforward) :
        (pid_output - scaled_feedforward);
}

static float encoder_motor_pid_apply_min_drive(
    float pwm_command, float speed_target, float speed_measurement,
    float forward_min_drive_pwm, float reverse_min_drive_pwm,
    float reference_pps)
{
    float abs_pwm;
    float min_drive_pwm;
    float scaled_min_drive;
    float hold_scale;

    if (speed_target == 0.0f) {
        return pwm_command;
    }

    min_drive_pwm = (speed_target > 0.0f) ? forward_min_drive_pwm :
        reverse_min_drive_pwm;
    if (min_drive_pwm <= 0.0f) {
        return pwm_command;
    }
    if (((speed_target > 0.0f) && (pwm_command < 0.0f)) ||
        ((speed_target < 0.0f) && (pwm_command > 0.0f))) {
        return pwm_command;
    }

    scaled_min_drive = min_drive_pwm;
    if (reference_pps > 0.0f) {
        hold_scale = 1.0f -
            (encoder_motor_pid_abs(speed_measurement) / reference_pps);
        if (hold_scale < 0.0f) {
            hold_scale = 0.0f;
        } else if (hold_scale > 1.0f) {
            hold_scale = 1.0f;
        }

        scaled_min_drive = min_drive_pwm * hold_scale;
        if (scaled_min_drive > min_drive_pwm) {
            scaled_min_drive = min_drive_pwm;
        }
    }

    abs_pwm = encoder_motor_pid_abs(pwm_command);
    if (abs_pwm >= scaled_min_drive) {
        return pwm_command;
    }

    return (speed_target > 0.0f) ? scaled_min_drive : -scaled_min_drive;
}

static float encoder_motor_pid_limit_to_target_direction(
    float pwm_command, float speed_target)
{
    if (speed_target > 0.0f) {
        return (pwm_command < 0.0f) ? 0.0f : pwm_command;
    }
    if (speed_target < 0.0f) {
        return (pwm_command > 0.0f) ? 0.0f : pwm_command;
    }
    return 0.0f;
}

static void encoder_motor_pid_apply_pwm(
    motor_id_t motor_id, int pwm_command)
{
    if (pwm_command > MOTOR_PWM_MAX) {
        pwm_command = MOTOR_PWM_MAX;
    } else if (pwm_command < -MOTOR_PWM_MAX) {
        pwm_command = -MOTOR_PWM_MAX;
    }

    if (motor_id == MOTOR_LEFT) {
        Motor_SetLeft(pwm_command);
    } else if (motor_id == MOTOR_RIGHT) {
        Motor_SetRight(pwm_command);
    }
}
