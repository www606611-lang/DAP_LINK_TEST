#ifndef PID_ENCODER_SPEED_CONTROL_H
#define PID_ENCODER_SPEED_CONTROL_H

#include "encoder.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} encoder_speed_control_pid_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float feedforward_pwm;
    float feedforward_reference_pps;
    float forward_min_drive_pwm;
    float reverse_min_drive_pwm;
    float min_drive_reference_pps;
} encoder_speed_control_config_t;

void EncoderSpeedControl_Init(uint32_t now_ms);
void EncoderSpeedControl_Task(uint32_t now_ms);
bool EncoderSpeedControl_IsInitialized(void);

void EncoderSpeedControl_SetTargetPps(
    float left_speed_pps, float right_speed_pps);
void EncoderSpeedControl_GetTargetPps(
    float *left_speed_pps, float *right_speed_pps);
void EncoderSpeedControl_GetPwmCommand(
    float *left_pwm_command, float *right_pwm_command);
void EncoderSpeedControl_SetSpeedTunings(
    encoder_id_t id, float kp, float ki, float kd);
void EncoderSpeedControl_SetSpeedOutputLimits(
    encoder_id_t id, float output_min, float output_max);
void EncoderSpeedControl_SetSpeedIntegralLimits(
    encoder_id_t id, float integral_min, float integral_max);
void EncoderSpeedControl_SetSpeedDeadband(
    encoder_id_t id, float deadband);
void EncoderSpeedControl_SetSpeedFeedforwardPwm(
    encoder_id_t id, float feedforward_pwm);
void EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
    encoder_id_t id, float reference_pps);
void EncoderSpeedControl_SetSpeedMinDriveConfig(
    encoder_id_t id, float forward_pwm, float reverse_pwm,
    float reference_pps);
void EncoderSpeedControl_SetMinDrivePwm(
    float left_pwm, float right_pwm, float reference_pps);
void EncoderSpeedControl_SetDirectionalMinDrivePwm(
    float left_forward_pwm, float left_reverse_pwm,
    float right_forward_pwm, float right_reverse_pwm,
    float reference_pps);
void EncoderSpeedControl_ClearMinDrivePwm(void);
void EncoderSpeedControl_SetOutputLimits(float left_pwm_limit,
    float right_pwm_limit);
void EncoderSpeedControl_RestoreDefaultOutputLimits(void);
void EncoderSpeedControl_GetSpeedTunings(
    encoder_speed_control_pid_t *left, encoder_speed_control_pid_t *right);
void EncoderSpeedControl_GetSpeedConfig(
    encoder_id_t id, encoder_speed_control_config_t *config);
void EncoderSpeedControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
