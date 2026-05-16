#ifndef MODULES_LINE_SENSOR_I2C_H
#define MODULES_LINE_SENSOR_I2C_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_SENSOR_I2C_CHANNEL_COUNT 8U

typedef struct {
    uint8_t raw;
    uint8_t active_mask;
    uint8_t active_count;
    bool channel[LINE_SENSOR_I2C_CHANNEL_COUNT];
} line_sensor_i2c_state_t;

void LineSensorI2C_Init(void);
bool LineSensorI2C_ReadRaw(uint8_t *raw);
bool LineSensorI2C_ReadState(line_sensor_i2c_state_t *state);
bool LineSensorI2C_SetCalibration(bool enabled);
uint8_t LineSensorI2C_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif
