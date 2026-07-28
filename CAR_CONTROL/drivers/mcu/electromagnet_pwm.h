#ifndef DRIVERS_MCU_ELECTROMAGNET_PWM_H
#define DRIVERS_MCU_ELECTROMAGNET_PWM_H

#include <stdbool.h>
#include <stdint.h>

#define ELECTROMAGNET_PWM_DUTY_MAX 1000U

void ElectromagnetPwm_ForceGpioLow(void);
void ElectromagnetPwm_Init(void);
bool ElectromagnetPwm_SetDuty(uint16_t duty_permille);
uint16_t ElectromagnetPwm_GetDuty(void);

#endif
