#ifndef DRIVERS_DEVICE_AT8236_MOTOR_H
#define DRIVERS_DEVICE_AT8236_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define AT8236_MOTOR_COMMAND_MAX 1000

typedef enum {
    AT8236_MOTOR_A = 0,
    AT8236_MOTOR_B,
    AT8236_MOTOR_COUNT
} at8236_motor_id_t;

void AT8236_MotorInit(void);
void AT8236_MotorSetInverted(at8236_motor_id_t id, bool inverted);
bool AT8236_MotorSetCommand(
    at8236_motor_id_t id, int16_t command_permille);
void AT8236_MotorStop(at8236_motor_id_t id);
void AT8236_MotorStopAll(void);
int16_t AT8236_MotorGetCommand(at8236_motor_id_t id);

#endif
