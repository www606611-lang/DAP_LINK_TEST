#include "icm20948.h"

#include "delay.h"
#include "i2c0_polling.h"
#include "imu_attitude_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ICM20948_REG_BANK_SEL       0x7FU
#define ICM20948_WHO_AM_I           0x00U
#define ICM20948_LP_CONFIG          0x05U
#define ICM20948_PWR_MGMT_1         0x06U
#define ICM20948_PWR_MGMT_2         0x07U
#define ICM20948_ACCEL_XOUT_H       0x2DU
#define ICM20948_GYRO_SMPLRT_DIV    0x00U
#define ICM20948_GYRO_CONFIG_1      0x01U
#define ICM20948_ODR_ALIGN_EN       0x09U
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10U
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11U
#define ICM20948_ACCEL_CONFIG       0x14U

#define ICM20948_ADDRESS_AD0_LOW    0x68U
#define ICM20948_ADDRESS_AD0_HIGH   0x69U
#define ICM20948_ACCEL_SENS_2G      16384.0f
#define ICM20948_GYRO_SENS_500DPS      65.5f
#define ICM20948_TEMP_SENS             333.87f
#define ICM20948_TEMP_OFFSET            21.0f
#define ICM20948_GYRO_CONFIG_500DPS     0x1BU
#define ICM20948_CALIBRATION_SAMPLES  400U
#define ICM20948_RETRY_INTERVAL_MS    1000U
#define ICM20948_MAX_READ_ERRORS         3U
#define ICM20948_DT_CLAMP_MS            20U
#define ICM20948_GYRO_DEADBAND_XY_DPS    0.05f
#define ICM20948_GYRO_DEADBAND_Z_DPS     0.12f
#define ICM20948_STILL_GYRO_XY_DPS       0.60f
#define ICM20948_STILL_GYRO_Z_DPS        0.80f
#define ICM20948_STILL_ACCEL_MIN_G2      0.90f
#define ICM20948_STILL_ACCEL_MAX_G2      1.10f
#define ICM20948_STILL_COUNT_MIN        20U
#define ICM20948_BIAS_TRACK_TAU_XY_S     2.0f
#define ICM20948_BIAS_TRACK_TAU_Z_S      0.8f
#define ICM20948_BIAS_TRACK_MAX_GAIN_XY  0.02f
#define ICM20948_BIAS_TRACK_MAX_GAIN_Z   0.05f

static icm20948_snapshot_t g_snapshot;
static uint8_t g_current_bank;
static uint16_t g_still_count;
static uint32_t g_last_update_ms;
static uint32_t g_next_retry_ms;

static icm20948_result_t icm20948_try_initialize(uint32_t now_ms);
static icm20948_result_t icm20948_probe(uint8_t address7, uint8_t *id);
static icm20948_result_t icm20948_switch_bank(uint8_t bank);
static icm20948_result_t icm20948_write_reg(uint8_t reg, uint8_t value);
static icm20948_result_t icm20948_read_regs(
    uint8_t reg, uint8_t *data, uint16_t length);
static icm20948_result_t icm20948_read_raw(icm20948_raw_t *raw);
static icm20948_result_t icm20948_read_data(icm20948_data_t *data);
static icm20948_result_t icm20948_calibrate_gyro(void);
static void icm20948_update_attitude(float dt_s);
static void icm20948_copy_attitude_snapshot(void);
static bool icm20948_is_stationary(const icm20948_data_t *data);
static void icm20948_track_gyro_bias(
    const icm20948_data_t *data, float dt_s);
static float icm20948_deadband(float value, float deadband);
static bool icm20948_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void ICM20948_Init(uint32_t now_ms)
{
    (void) memset(&g_snapshot, 0, sizeof(g_snapshot));
    g_snapshot.address7 = ICM20948_ADDRESS_AD0_HIGH;
    g_snapshot.who_am_i = 0xFFU;
    g_snapshot.state = ICM20948_STATE_OFFLINE;
    g_snapshot.last_result = ICM20948_I2C_ERROR;
    g_current_bank = 0xFFU;
    g_still_count = 0U;
    g_last_update_ms = now_ms;
    g_next_retry_ms = now_ms;

    I2C0Polling_Init();
    g_snapshot.last_result = icm20948_try_initialize(now_ms);
}

void ICM20948_Task(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!g_snapshot.ready) {
        if (icm20948_deadline_reached(now_ms, g_next_retry_ms)) {
            g_snapshot.last_result = icm20948_try_initialize(now_ms);
        }
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < ICM20948_UPDATE_INTERVAL_MS) {
        return;
    }
    if (elapsed_ms > ICM20948_DT_CLAMP_MS) {
        elapsed_ms = ICM20948_UPDATE_INTERVAL_MS;
    }
    g_last_update_ms = now_ms;

    if (icm20948_read_data(&g_snapshot.data) != ICM20948_OK) {
        g_snapshot.read_error_count++;
        if (g_snapshot.consecutive_read_errors < UINT8_MAX) {
            g_snapshot.consecutive_read_errors++;
        }
        g_snapshot.last_result = ICM20948_READ_ERROR;
        if (g_snapshot.consecutive_read_errors >=
            ICM20948_MAX_READ_ERRORS) {
            g_snapshot.ready = false;
            g_snapshot.state = ICM20948_STATE_OFFLINE;
            g_next_retry_ms = now_ms + ICM20948_RETRY_INTERVAL_MS;
        }
        return;
    }

    g_snapshot.consecutive_read_errors = 0U;
    g_snapshot.last_result = ICM20948_OK;
    g_snapshot.sample_count++;
    g_snapshot.last_sample_ms = now_ms;
    icm20948_update_attitude((float) elapsed_ms / 1000.0f);
}

void ICM20948_ResetYaw(void)
{
    ImuAttitudeEstimator_ResetYaw();
    icm20948_copy_attitude_snapshot();
    g_snapshot.yaw_deg = 0.0f;
}

bool ICM20948_IsReady(void)
{
    return g_snapshot.ready;
}

bool ICM20948_GetSnapshot(icm20948_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static icm20948_result_t icm20948_try_initialize(uint32_t now_ms)
{
    uint8_t id = 0xFFU;
    uint8_t register_value;
    imu_attitude_input_t attitude_input;

    g_snapshot.ready = false;
    g_snapshot.state = ICM20948_STATE_CALIBRATING;
    g_snapshot.who_am_i = 0xFFU;
    g_current_bank = 0xFFU;
    I2C0Polling_Recover();
    delay_ms(20U);

    if (icm20948_probe(ICM20948_ADDRESS_AD0_HIGH, &id) == ICM20948_OK) {
        g_snapshot.address7 = ICM20948_ADDRESS_AD0_HIGH;
    } else if (icm20948_probe(
            ICM20948_ADDRESS_AD0_LOW, &id) == ICM20948_OK) {
        g_snapshot.address7 = ICM20948_ADDRESS_AD0_LOW;
    } else {
        g_snapshot.state = ICM20948_STATE_OFFLINE;
        g_snapshot.last_result = ICM20948_ID_ERROR;
        g_next_retry_ms = now_ms + ICM20948_RETRY_INTERVAL_MS;
        return g_snapshot.last_result;
    }
    g_snapshot.who_am_i = id;

    g_current_bank = 0xFFU;
    if (icm20948_switch_bank(0U) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_PWR_MGMT_1, 0x80U) != ICM20948_OK) {
        goto config_error;
    }
    delay_ms(50U);
    g_current_bank = 0xFFU;

    if (icm20948_switch_bank(0U) != ICM20948_OK ||
        icm20948_read_regs(ICM20948_PWR_MGMT_1,
            &register_value, 1U) != ICM20948_OK) {
        goto config_error;
    }
    register_value &= (uint8_t) ~0x47U;
    register_value |= 0x01U;
    if (icm20948_write_reg(
            ICM20948_PWR_MGMT_1, register_value) != ICM20948_OK) {
        goto config_error;
    }
    delay_ms(10U);

    if (icm20948_write_reg(ICM20948_PWR_MGMT_2, 0x00U) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_LP_CONFIG, 0x00U) != ICM20948_OK ||
        icm20948_switch_bank(2U) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_ODR_ALIGN_EN, 0x01U) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_GYRO_SMPLRT_DIV, 0x04U) != ICM20948_OK ||
        icm20948_write_reg(
            ICM20948_ACCEL_SMPLRT_DIV_1, 0x00U) != ICM20948_OK ||
        icm20948_write_reg(
            ICM20948_ACCEL_SMPLRT_DIV_2, 0x04U) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_GYRO_CONFIG_1,
            ICM20948_GYRO_CONFIG_500DPS) != ICM20948_OK ||
        icm20948_write_reg(ICM20948_ACCEL_CONFIG, 0x00U) != ICM20948_OK ||
        icm20948_switch_bank(0U) != ICM20948_OK) {
        goto config_error;
    }

    g_snapshot.gyro_bias_x_dps = 0.0f;
    g_snapshot.gyro_bias_y_dps = 0.0f;
    g_snapshot.gyro_bias_z_dps = 0.0f;
    if (icm20948_calibrate_gyro() != ICM20948_OK) {
        g_snapshot.state = ICM20948_STATE_OFFLINE;
        g_snapshot.last_result = ICM20948_CALIBRATION_ERROR;
        g_next_retry_ms = now_ms + ICM20948_RETRY_INTERVAL_MS;
        return g_snapshot.last_result;
    }
    if (icm20948_read_data(&g_snapshot.data) != ICM20948_OK) {
        goto config_error;
    }

    attitude_input.ax_g = g_snapshot.data.ax_g;
    attitude_input.ay_g = g_snapshot.data.ay_g;
    attitude_input.az_g = g_snapshot.data.az_g;
    attitude_input.gx_dps = 0.0f;
    attitude_input.gy_dps = 0.0f;
    attitude_input.gz_dps = 0.0f;
    if (!ImuAttitudeEstimator_Init(&attitude_input)) {
        goto config_error;
    }
    icm20948_copy_attitude_snapshot();
    g_still_count = 0U;
    g_snapshot.consecutive_read_errors = 0U;
    g_snapshot.last_sample_ms = now_ms;
    g_snapshot.ready = true;
    g_snapshot.state = ICM20948_STATE_READY;
    g_snapshot.last_result = ICM20948_OK;
    g_last_update_ms = now_ms;
    return ICM20948_OK;

config_error:
    g_snapshot.state = ICM20948_STATE_OFFLINE;
    g_snapshot.last_result = ICM20948_CONFIG_ERROR;
    g_next_retry_ms = now_ms + ICM20948_RETRY_INTERVAL_MS;
    return g_snapshot.last_result;
}

static icm20948_result_t icm20948_probe(uint8_t address7, uint8_t *id)
{
    uint8_t old_address = g_snapshot.address7;
    icm20948_result_t result;

    if (id == NULL) {
        return ICM20948_BAD_ARGUMENT;
    }
    g_snapshot.address7 = address7;
    g_current_bank = 0xFFU;
    result = icm20948_switch_bank(0U);
    if (result == ICM20948_OK) {
        result = icm20948_read_regs(ICM20948_WHO_AM_I, id, 1U);
    }
    if ((result == ICM20948_OK) &&
        (*id != ICM20948_WHO_AM_I_VALUE)) {
        result = ICM20948_ID_ERROR;
    }
    if (result != ICM20948_OK) {
        g_snapshot.address7 = old_address;
        g_current_bank = 0xFFU;
    }
    return result;
}

static icm20948_result_t icm20948_switch_bank(uint8_t bank)
{
    if (g_current_bank == bank) {
        return ICM20948_OK;
    }
    if (icm20948_write_reg(
            ICM20948_REG_BANK_SEL, (uint8_t) (bank << 4)) != ICM20948_OK) {
        return ICM20948_I2C_ERROR;
    }
    g_current_bank = bank;
    delay_ms(1U);
    return ICM20948_OK;
}

static icm20948_result_t icm20948_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};

    return (I2C0Polling_Write(
        g_snapshot.address7, packet, 2U) == I2C0_POLLING_OK) ?
        ICM20948_OK : ICM20948_I2C_ERROR;
}

static icm20948_result_t icm20948_read_regs(
    uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return ICM20948_BAD_ARGUMENT;
    }
    return (I2C0Polling_ReadRegister(
        g_snapshot.address7, reg, data, length) == I2C0_POLLING_OK) ?
        ICM20948_OK : ICM20948_I2C_ERROR;
}

static icm20948_result_t icm20948_read_raw(icm20948_raw_t *raw)
{
    uint8_t data[14];

    if (raw == NULL) {
        return ICM20948_BAD_ARGUMENT;
    }
    if ((icm20948_switch_bank(0U) != ICM20948_OK) ||
        (icm20948_read_regs(
            ICM20948_ACCEL_XOUT_H, data, sizeof(data)) != ICM20948_OK)) {
        return ICM20948_READ_ERROR;
    }

    raw->ax = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    raw->ay = (int16_t) (((uint16_t) data[2] << 8) | data[3]);
    raw->az = (int16_t) (((uint16_t) data[4] << 8) | data[5]);
    raw->gx = (int16_t) (((uint16_t) data[6] << 8) | data[7]);
    raw->gy = (int16_t) (((uint16_t) data[8] << 8) | data[9]);
    raw->gz = (int16_t) (((uint16_t) data[10] << 8) | data[11]);
    raw->temperature =
        (int16_t) (((uint16_t) data[12] << 8) | data[13]);
    return ICM20948_OK;
}

static icm20948_result_t icm20948_read_data(icm20948_data_t *data)
{
    icm20948_raw_t raw;

    if (data == NULL) {
        return ICM20948_BAD_ARGUMENT;
    }
    if (icm20948_read_raw(&raw) != ICM20948_OK) {
        return ICM20948_READ_ERROR;
    }

    data->ax_g = (float) raw.ax / ICM20948_ACCEL_SENS_2G;
    data->ay_g = (float) raw.ay / ICM20948_ACCEL_SENS_2G;
    data->az_g = (float) raw.az / ICM20948_ACCEL_SENS_2G;
    data->gx_dps = (float) raw.gx / ICM20948_GYRO_SENS_500DPS -
        g_snapshot.gyro_bias_x_dps;
    data->gy_dps = (float) raw.gy / ICM20948_GYRO_SENS_500DPS -
        g_snapshot.gyro_bias_y_dps;
    data->gz_dps = (float) raw.gz / ICM20948_GYRO_SENS_500DPS -
        g_snapshot.gyro_bias_z_dps;
    data->temperature_c =
        (float) raw.temperature / ICM20948_TEMP_SENS +
        ICM20948_TEMP_OFFSET;
    return ICM20948_OK;
}

static icm20948_result_t icm20948_calibrate_gyro(void)
{
    uint16_t index;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    icm20948_raw_t raw;

    for (index = 0U; index < ICM20948_CALIBRATION_SAMPLES; index++) {
        if (icm20948_read_raw(&raw) != ICM20948_OK) {
            return ICM20948_CALIBRATION_ERROR;
        }
        sum_x += (float) raw.gx / ICM20948_GYRO_SENS_500DPS;
        sum_y += (float) raw.gy / ICM20948_GYRO_SENS_500DPS;
        sum_z += (float) raw.gz / ICM20948_GYRO_SENS_500DPS;
        delay_ms(2U);
    }

    g_snapshot.gyro_bias_x_dps =
        sum_x / (float) ICM20948_CALIBRATION_SAMPLES;
    g_snapshot.gyro_bias_y_dps =
        sum_y / (float) ICM20948_CALIBRATION_SAMPLES;
    g_snapshot.gyro_bias_z_dps =
        sum_z / (float) ICM20948_CALIBRATION_SAMPLES;
    return ICM20948_OK;
}

static void icm20948_update_attitude(float dt_s)
{
    float gx_dps = g_snapshot.data.gx_dps;
    float gy_dps = g_snapshot.data.gy_dps;
    float gz_dps = g_snapshot.data.gz_dps;
    bool stationary_locked = false;
    imu_attitude_input_t attitude_input;

    if (icm20948_is_stationary(&g_snapshot.data)) {
        icm20948_track_gyro_bias(&g_snapshot.data, dt_s);
        if (g_still_count < ICM20948_STILL_COUNT_MIN) {
            g_still_count++;
        }
        if (g_still_count >= ICM20948_STILL_COUNT_MIN) {
            stationary_locked = true;
            gx_dps = 0.0f;
            gy_dps = 0.0f;
            gz_dps = 0.0f;
        }
    } else {
        g_still_count = 0U;
    }

    gx_dps = icm20948_deadband(gx_dps, ICM20948_GYRO_DEADBAND_XY_DPS);
    gy_dps = icm20948_deadband(gy_dps, ICM20948_GYRO_DEADBAND_XY_DPS);
    gz_dps = icm20948_deadband(gz_dps, ICM20948_GYRO_DEADBAND_Z_DPS);

    attitude_input.ax_g = g_snapshot.data.ax_g;
    attitude_input.ay_g = g_snapshot.data.ay_g;
    attitude_input.az_g = g_snapshot.data.az_g;
    attitude_input.gx_dps = gx_dps;
    attitude_input.gy_dps = gy_dps;
    attitude_input.gz_dps = gz_dps;
    g_snapshot.stationary = stationary_locked;
    g_snapshot.attitude_valid = ImuAttitudeEstimator_Update(
        &attitude_input, dt_s, stationary_locked);
    icm20948_copy_attitude_snapshot();
}

static void icm20948_copy_attitude_snapshot(void)
{
    imu_attitude_snapshot_t attitude;

    if (!ImuAttitudeEstimator_GetSnapshot(&attitude)) {
        g_snapshot.attitude_valid = false;
        return;
    }
    g_snapshot.roll_deg = attitude.roll_deg;
    g_snapshot.pitch_deg = attitude.pitch_deg;
    g_snapshot.yaw_deg = attitude.yaw_deg;
    g_snapshot.yaw_rate_dps = attitude.yaw_rate_dps;
    g_snapshot.accel_norm_g = attitude.accel_norm_g;
    g_snapshot.quaternion_w = attitude.quaternion_w;
    g_snapshot.quaternion_x = attitude.quaternion_x;
    g_snapshot.quaternion_y = attitude.quaternion_y;
    g_snapshot.quaternion_z = attitude.quaternion_z;
    g_snapshot.attitude_valid = attitude.valid;
}

static bool icm20948_is_stationary(const icm20948_data_t *data)
{
    float accel_g2;

    if ((fabsf(data->gx_dps) > ICM20948_STILL_GYRO_XY_DPS) ||
        (fabsf(data->gy_dps) > ICM20948_STILL_GYRO_XY_DPS) ||
        (fabsf(data->gz_dps) > ICM20948_STILL_GYRO_Z_DPS)) {
        return false;
    }
    accel_g2 = data->ax_g * data->ax_g + data->ay_g * data->ay_g +
        data->az_g * data->az_g;
    return (accel_g2 >=
            ICM20948_STILL_ACCEL_MIN_G2 * ICM20948_STILL_ACCEL_MIN_G2) &&
        (accel_g2 <=
            ICM20948_STILL_ACCEL_MAX_G2 * ICM20948_STILL_ACCEL_MAX_G2);
}

static void icm20948_track_gyro_bias(
    const icm20948_data_t *data, float dt_s)
{
    float gain_xy = dt_s / ICM20948_BIAS_TRACK_TAU_XY_S;
    float gain_z = dt_s / ICM20948_BIAS_TRACK_TAU_Z_S;

    if (gain_xy > ICM20948_BIAS_TRACK_MAX_GAIN_XY) {
        gain_xy = ICM20948_BIAS_TRACK_MAX_GAIN_XY;
    }
    if (gain_z > ICM20948_BIAS_TRACK_MAX_GAIN_Z) {
        gain_z = ICM20948_BIAS_TRACK_MAX_GAIN_Z;
    }
    g_snapshot.gyro_bias_x_dps += data->gx_dps * gain_xy;
    g_snapshot.gyro_bias_y_dps += data->gy_dps * gain_xy;
    g_snapshot.gyro_bias_z_dps += data->gz_dps * gain_z;
}

static float icm20948_deadband(float value, float deadband)
{
    return (fabsf(value) < deadband) ? 0.0f : value;
}

static bool icm20948_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}
