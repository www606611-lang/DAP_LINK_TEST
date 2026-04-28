#ifndef PID_PID_H
#define PID_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float prev_measurement;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;

    bool initialized;
} pid_controller_t;

void PID_Init(pid_controller_t *pid);
void PID_Reset(pid_controller_t *pid);

void PID_SetTunings(pid_controller_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimits(
    pid_controller_t *pid, float output_min, float output_max);
void PID_SetIntegralLimits(
    pid_controller_t *pid, float integral_min, float integral_max);
void PID_SetDeadband(pid_controller_t *pid, float deadband);

float PID_Update(pid_controller_t *pid, float setpoint, float measurement,
    float dt_s);
float PID_UpdateError(pid_controller_t *pid, float error, float measurement,
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif
