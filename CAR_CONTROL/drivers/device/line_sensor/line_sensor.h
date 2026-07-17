#ifndef DRIVERS_DEVICE_LINE_SENSOR_H
#define DRIVERS_DEVICE_LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_CHANNEL_COUNT 8U

typedef enum {
    LINE_SENSOR_STATE_UNINITIALIZED = 0,
    LINE_SENSOR_STATE_BOOT_WAIT,
    LINE_SENSOR_STATE_CALIBRATION_ON,
    LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT,
    LINE_SENSOR_STATE_READY,
    LINE_SENSOR_STATE_ERROR
} line_sensor_state_t;

typedef enum {
    LINE_SENSOR_OK = 0,
    LINE_SENSOR_BAD_ARGUMENT,
    LINE_SENSOR_BUSY,
    LINE_SENSOR_I2C_ERROR
} line_sensor_result_t;

typedef struct {
    bool channel[LINE_SENSOR_CHANNEL_COUNT];
    uint8_t raw;
    uint8_t active_mask;
    uint8_t active_count;
    int16_t line_error;
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t calibration_count;
    uint32_t last_sample_ms;
    line_sensor_state_t state;
    line_sensor_result_t last_result;
    bool line_seen;
    bool ready;
} line_sensor_snapshot_t;

void LineSensor_Init(uint32_t now_ms);
void LineSensor_Task(uint32_t now_ms);
line_sensor_result_t LineSensor_RequestCalibration(uint32_t now_ms);
bool LineSensor_GetSnapshot(line_sensor_snapshot_t *snapshot);
const char *LineSensor_GetStateText(void);

#endif
