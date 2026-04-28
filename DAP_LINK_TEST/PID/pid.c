#include "pid.h"

#include <stddef.h>

static float pid_clamp(float value, float min_value, float max_value);
static float pid_abs(float value);

void PID_Init(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;

    pid->output_min = -1.0e9f;
    pid->output_max = 1.0e9f;
    pid->integral_min = -1.0e9f;
    pid->integral_max = 1.0e9f;
    pid->deadband = 0.0f;

    pid->initialized = false;
}

void PID_Reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = false;
}

void PID_SetTunings(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(
    pid_controller_t *pid, float output_min, float output_max)
{
    if ((pid == NULL) || (output_min > output_max)) {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;
}

void PID_SetIntegralLimits(
    pid_controller_t *pid, float integral_min, float integral_max)
{
    if ((pid == NULL) || (integral_min > integral_max)) {
        return;
    }

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

void PID_SetDeadband(pid_controller_t *pid, float deadband)
{
    if (pid == NULL) {
        return;
    }

    pid->deadband = (deadband < 0.0f) ? 0.0f : deadband;
}

float PID_Update(pid_controller_t *pid, float setpoint, float measurement,
    float dt_s)
{
    return PID_UpdateError(pid, setpoint - measurement, measurement, dt_s);
}

float PID_UpdateError(pid_controller_t *pid, float error, float measurement,
    float dt_s)
{
    float derivative = 0.0f;
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    if (pid_abs(error) <= pid->deadband) {
        error = 0.0f;
    }

    if (!pid->initialized) {
        pid->prev_error = error;
        pid->prev_measurement = measurement;
        pid->initialized = true;
    }

    pid->integral += error * dt_s;
    pid->integral = pid_clamp(
        pid->integral, pid->integral_min, pid->integral_max);

    derivative = -(measurement - pid->prev_measurement) / dt_s;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = pid_clamp(output, pid->output_min, pid->output_max);

    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}

static float pid_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float pid_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}
