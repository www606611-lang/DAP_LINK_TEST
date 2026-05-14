#include "encoder_motor_pid.h"

#include <stddef.h>

static int encoder_motor_pid_round_to_int(float value);
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
