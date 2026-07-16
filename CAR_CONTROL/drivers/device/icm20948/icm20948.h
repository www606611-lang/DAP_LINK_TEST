#ifndef DRIVERS_DEVICE_ICM20948_H
#define DRIVERS_DEVICE_ICM20948_H

#include <stdbool.h>
#include <stdint.h>

#define ICM20948_UPDATE_INTERVAL_MS 10U
#define ICM20948_WHO_AM_I_VALUE     0xEAU

typedef enum {
    ICM20948_STATE_OFFLINE = 0,
    ICM20948_STATE_CALIBRATING,
    ICM20948_STATE_READY
} icm20948_state_t;

typedef enum {
    ICM20948_OK = 0,
    ICM20948_I2C_ERROR,
    ICM20948_ID_ERROR,
    ICM20948_CONFIG_ERROR,
    ICM20948_CALIBRATION_ERROR,
    ICM20948_READ_ERROR,
    ICM20948_BAD_ARGUMENT
} icm20948_result_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temperature;
} icm20948_raw_t;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temperature_c;
} icm20948_data_t;

typedef struct {
    icm20948_data_t data;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t last_sample_ms;
    uint8_t address7;
    uint8_t who_am_i;
    uint8_t consecutive_read_errors;
    icm20948_state_t state;
    icm20948_result_t last_result;
    bool ready;
} icm20948_snapshot_t;

void ICM20948_Init(uint32_t now_ms);
void ICM20948_Task(uint32_t now_ms);
void ICM20948_ResetYaw(void);
bool ICM20948_IsReady(void);
bool ICM20948_GetSnapshot(icm20948_snapshot_t *snapshot);

#endif
