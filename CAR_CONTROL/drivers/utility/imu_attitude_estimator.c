#include "imu_attitude_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_ATTITUDE_PI                  3.14159265358979323846f
#define IMU_ATTITUDE_DEG_TO_RAD          (IMU_ATTITUDE_PI / 180.0f)
#define IMU_ATTITUDE_RAD_TO_DEG          (180.0f / IMU_ATTITUDE_PI)
#define IMU_ATTITUDE_ACCEL_CORRECTION_KP 1.8f
#define IMU_ATTITUDE_ACCEL_FULL_ERROR_G  0.08f
#define IMU_ATTITUDE_ACCEL_ZERO_ERROR_G  0.25f
#define IMU_ATTITUDE_MIN_ACCEL_NORM_G    0.20f
#define IMU_ATTITUDE_MIN_QUATERNION_NORM 0.000001f

static imu_attitude_snapshot_t g_attitude;
static float g_last_quaternion_yaw_deg;

static bool imu_attitude_accel_angles(const imu_attitude_input_t *input,
    float *roll_rad, float *pitch_rad, float *norm_g);
static void imu_attitude_update_euler(void);
static float imu_attitude_accel_trust(float norm_g);
static float imu_attitude_clamp(float value, float minimum, float maximum);
static float imu_attitude_wrap_deg(float angle_deg);

bool ImuAttitudeEstimator_Init(const imu_attitude_input_t *input)
{
    float roll_rad;
    float pitch_rad;
    float norm_g;
    float half_roll;
    float half_pitch;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;

    (void) memset(&g_attitude, 0, sizeof(g_attitude));
    if (!imu_attitude_accel_angles(
            input, &roll_rad, &pitch_rad, &norm_g)) {
        return false;
    }

    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    sin_roll = sinf(half_roll);
    cos_roll = cosf(half_roll);
    sin_pitch = sinf(half_pitch);
    cos_pitch = cosf(half_pitch);

    g_attitude.quaternion_w = cos_roll * cos_pitch;
    g_attitude.quaternion_x = sin_roll * cos_pitch;
    g_attitude.quaternion_y = cos_roll * sin_pitch;
    g_attitude.quaternion_z = -sin_roll * sin_pitch;
    g_attitude.accel_norm_g = norm_g;
    g_attitude.valid = true;
    imu_attitude_update_euler();
    g_last_quaternion_yaw_deg = g_attitude.yaw_deg;
    g_attitude.yaw_deg = 0.0f;
    g_attitude.yaw_rate_dps = 0.0f;
    return true;
}

bool ImuAttitudeEstimator_Update(const imu_attitude_input_t *input,
    float dt_s, bool freeze_yaw)
{
    float ax;
    float ay;
    float az;
    float accel_norm;
    float accel_inverse;
    float accel_trust;
    float half_vx;
    float half_vy;
    float half_vz;
    float half_ex;
    float half_ey;
    float half_ez;
    float gx_rad;
    float gy_rad;
    float gz_rad;
    float old_w;
    float old_x;
    float old_y;
    float old_z;
    float half_dt;
    float quaternion_norm;
    float quaternion_inverse;
    float public_yaw_deg;
    float quaternion_yaw_deg;
    float yaw_delta_deg;

    if ((input == NULL) || (dt_s <= 0.0f) || (dt_s > 0.1f)) {
        return false;
    }
    if (!g_attitude.valid && !ImuAttitudeEstimator_Init(input)) {
        return false;
    }

    ax = input->ax_g;
    ay = input->ay_g;
    az = input->az_g;
    accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    accel_trust = imu_attitude_accel_trust(accel_norm);
    if (accel_norm >= IMU_ATTITUDE_MIN_ACCEL_NORM_G) {
        accel_inverse = 1.0f / accel_norm;
        ax *= accel_inverse;
        ay *= accel_inverse;
        az *= accel_inverse;
    } else {
        accel_trust = 0.0f;
    }

    old_w = g_attitude.quaternion_w;
    old_x = g_attitude.quaternion_x;
    old_y = g_attitude.quaternion_y;
    old_z = g_attitude.quaternion_z;

    half_vx = old_x * old_z - old_w * old_y;
    half_vy = old_w * old_x + old_y * old_z;
    half_vz = old_w * old_w - 0.5f + old_z * old_z;
    half_ex = ay * half_vz - az * half_vy;
    half_ey = az * half_vx - ax * half_vz;
    half_ez = ax * half_vy - ay * half_vx;

    gx_rad = input->gx_dps * IMU_ATTITUDE_DEG_TO_RAD +
        2.0f * IMU_ATTITUDE_ACCEL_CORRECTION_KP * accel_trust * half_ex;
    gy_rad = input->gy_dps * IMU_ATTITUDE_DEG_TO_RAD +
        2.0f * IMU_ATTITUDE_ACCEL_CORRECTION_KP * accel_trust * half_ey;
    gz_rad = input->gz_dps * IMU_ATTITUDE_DEG_TO_RAD +
        2.0f * IMU_ATTITUDE_ACCEL_CORRECTION_KP * accel_trust * half_ez;

    half_dt = 0.5f * dt_s;
    g_attitude.quaternion_w +=
        (-old_x * gx_rad - old_y * gy_rad - old_z * gz_rad) * half_dt;
    g_attitude.quaternion_x +=
        (old_w * gx_rad + old_y * gz_rad - old_z * gy_rad) * half_dt;
    g_attitude.quaternion_y +=
        (old_w * gy_rad - old_x * gz_rad + old_z * gx_rad) * half_dt;
    g_attitude.quaternion_z +=
        (old_w * gz_rad + old_x * gy_rad - old_y * gx_rad) * half_dt;

    quaternion_norm = sqrtf(
        g_attitude.quaternion_w * g_attitude.quaternion_w +
        g_attitude.quaternion_x * g_attitude.quaternion_x +
        g_attitude.quaternion_y * g_attitude.quaternion_y +
        g_attitude.quaternion_z * g_attitude.quaternion_z);
    if (quaternion_norm < IMU_ATTITUDE_MIN_QUATERNION_NORM) {
        g_attitude.valid = false;
        return false;
    }
    quaternion_inverse = 1.0f / quaternion_norm;
    g_attitude.quaternion_w *= quaternion_inverse;
    g_attitude.quaternion_x *= quaternion_inverse;
    g_attitude.quaternion_y *= quaternion_inverse;
    g_attitude.quaternion_z *= quaternion_inverse;

    public_yaw_deg = g_attitude.yaw_deg;
    imu_attitude_update_euler();
    quaternion_yaw_deg = g_attitude.yaw_deg;
    yaw_delta_deg = imu_attitude_wrap_deg(
        quaternion_yaw_deg - g_last_quaternion_yaw_deg);
    g_last_quaternion_yaw_deg = quaternion_yaw_deg;
    if (freeze_yaw) {
        g_attitude.yaw_deg = public_yaw_deg;
        g_attitude.yaw_rate_dps = 0.0f;
    } else {
        g_attitude.yaw_deg = imu_attitude_wrap_deg(
            public_yaw_deg + yaw_delta_deg);
        g_attitude.yaw_rate_dps = yaw_delta_deg / dt_s;
    }
    g_attitude.accel_norm_g = accel_norm;
    g_attitude.update_count++;
    g_attitude.valid = true;
    return true;
}

void ImuAttitudeEstimator_ResetYaw(void)
{
    if (!g_attitude.valid) {
        return;
    }
    imu_attitude_update_euler();
    g_last_quaternion_yaw_deg = g_attitude.yaw_deg;
    g_attitude.yaw_deg = 0.0f;
    g_attitude.yaw_rate_dps = 0.0f;
}

bool ImuAttitudeEstimator_GetSnapshot(imu_attitude_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_attitude;
    return g_attitude.valid;
}

static bool imu_attitude_accel_angles(const imu_attitude_input_t *input,
    float *roll_rad, float *pitch_rad, float *norm_g)
{
    float norm;

    if ((input == NULL) || (roll_rad == NULL) || (pitch_rad == NULL) ||
        (norm_g == NULL)) {
        return false;
    }
    norm = sqrtf(input->ax_g * input->ax_g +
        input->ay_g * input->ay_g + input->az_g * input->az_g);
    if (norm < IMU_ATTITUDE_MIN_ACCEL_NORM_G) {
        return false;
    }
    *roll_rad = atan2f(input->ay_g, input->az_g);
    *pitch_rad = atan2f(-input->ax_g,
        sqrtf(input->ay_g * input->ay_g + input->az_g * input->az_g));
    *norm_g = norm;
    return true;
}

static void imu_attitude_update_euler(void)
{
    float w = g_attitude.quaternion_w;
    float x = g_attitude.quaternion_x;
    float y = g_attitude.quaternion_y;
    float z = g_attitude.quaternion_z;
    float pitch_sine = 2.0f * (w * y - z * x);

    g_attitude.roll_deg = atan2f(2.0f * (w * x + y * z),
        1.0f - 2.0f * (x * x + y * y)) * IMU_ATTITUDE_RAD_TO_DEG;
    g_attitude.pitch_deg = asinf(
        imu_attitude_clamp(pitch_sine, -1.0f, 1.0f)) *
        IMU_ATTITUDE_RAD_TO_DEG;
    g_attitude.yaw_deg = atan2f(2.0f * (w * z + x * y),
        1.0f - 2.0f * (y * y + z * z)) * IMU_ATTITUDE_RAD_TO_DEG;
}

static float imu_attitude_accel_trust(float norm_g)
{
    float error_g = fabsf(norm_g - 1.0f);

    if (error_g <= IMU_ATTITUDE_ACCEL_FULL_ERROR_G) {
        return 1.0f;
    }
    if (error_g >= IMU_ATTITUDE_ACCEL_ZERO_ERROR_G) {
        return 0.0f;
    }
    return 1.0f -
        (error_g - IMU_ATTITUDE_ACCEL_FULL_ERROR_G) /
        (IMU_ATTITUDE_ACCEL_ZERO_ERROR_G -
            IMU_ATTITUDE_ACCEL_FULL_ERROR_G);
}

static float imu_attitude_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float imu_attitude_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
