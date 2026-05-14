#ifndef PID_ENCODER_MOTOR_PID_H
#define PID_ENCODER_MOTOR_PID_H

#include "pid.h"
#include "encoder.h"
#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_MOTOR_PID_MODE_STOP = 0,
    ENCODER_MOTOR_PID_MODE_SPEED,
    ENCODER_MOTOR_PID_MODE_POSITION
} encoder_motor_pid_mode_t;

typedef struct {
    motor_id_t motor_id;
    encoder_id_t encoder_id;
    pid_controller_t speed_pid;
    pid_controller_t position_pid;
    encoder_snapshot_t snapshot;
    float target_speed_pps;
    float target_position_count;
    float cascade_speed_target_pps;
    float pwm_command;
    uint32_t last_update_ms;
    encoder_motor_pid_mode_t mode;
    bool enabled;
} encoder_motor_pid_t;

void EncoderMotorPID_Init(encoder_motor_pid_t *controller,
    motor_id_t motor_id, encoder_id_t encoder_id, uint32_t now_ms);
void EncoderMotorPID_Reset(encoder_motor_pid_t *controller, uint32_t now_ms);
void EncoderMotorPID_Stop(encoder_motor_pid_t *controller);

void EncoderMotorPID_SetSpeedTunings(encoder_motor_pid_t *controller,
    float kp, float ki, float kd);
void EncoderMotorPID_SetPositionTunings(encoder_motor_pid_t *controller,
    float kp, float ki, float kd);
void EncoderMotorPID_SetSpeedOutputLimits(encoder_motor_pid_t *controller,
    float output_min, float output_max);
void EncoderMotorPID_SetPositionOutputLimits(encoder_motor_pid_t *controller,
    float output_min, float output_max);
void EncoderMotorPID_SetSpeedIntegralLimits(encoder_motor_pid_t *controller,
    float integral_min, float integral_max);
void EncoderMotorPID_SetPositionIntegralLimits(encoder_motor_pid_t *controller,
    float integral_min, float integral_max);
void EncoderMotorPID_SetSpeedDeadband(encoder_motor_pid_t *controller,
    float deadband);
void EncoderMotorPID_SetPositionDeadband(encoder_motor_pid_t *controller,
    float deadband);

void EncoderMotorPID_SetTargetSpeedPps(encoder_motor_pid_t *controller,
    float target_speed_pps);
void EncoderMotorPID_SetTargetPositionCount(encoder_motor_pid_t *controller,
    float target_position_count);

bool EncoderMotorPID_Update(encoder_motor_pid_t *controller, uint32_t now_ms);

float EncoderMotorPID_GetPwmCommand(
    const encoder_motor_pid_t *controller);
float EncoderMotorPID_GetSpeedTargetPps(
    const encoder_motor_pid_t *controller);
float EncoderMotorPID_GetTargetPositionCount(
    const encoder_motor_pid_t *controller);
encoder_motor_pid_mode_t EncoderMotorPID_GetMode(
    const encoder_motor_pid_t *controller);
bool EncoderMotorPID_GetSnapshot(const encoder_motor_pid_t *controller,
    encoder_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
