#ifndef DRIVERS_UTILITY_IMU_ATTITUDE_ESTIMATOR_H
#define DRIVERS_UTILITY_IMU_ATTITUDE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} imu_attitude_input_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_rate_dps;
    float accel_norm_g;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    uint32_t update_count;
    bool valid;
} imu_attitude_snapshot_t;

bool ImuAttitudeEstimator_Init(const imu_attitude_input_t *input);
bool ImuAttitudeEstimator_Update(const imu_attitude_input_t *input,
    float dt_s, bool freeze_yaw);
void ImuAttitudeEstimator_ResetYaw(void);
bool ImuAttitudeEstimator_GetSnapshot(imu_attitude_snapshot_t *snapshot);

#endif
