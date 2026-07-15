#ifndef DRIVERS_MCU_MOTOR_PWM_H
#define DRIVERS_MCU_MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_DUTY_MAX 1000U

typedef enum {
    MOTOR_PWM_CHANNEL_A = 0,
    MOTOR_PWM_CHANNEL_B,
    MOTOR_PWM_CHANNEL_COUNT
} motor_pwm_channel_t;

void MotorPwm_Init(void);
bool MotorPwm_Enable(motor_pwm_channel_t channel);
void MotorPwm_Disable(motor_pwm_channel_t channel);
void MotorPwm_DisableAll(void);
bool MotorPwm_SetDuty(motor_pwm_channel_t channel,
    uint16_t in1_permille, uint16_t in2_permille);
bool MotorPwm_IsEnabled(motor_pwm_channel_t channel);

#endif
