#include "icm20948.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stddef.h>

#define ICM20948_REG_BANK_SEL          0x7FU

#define ICM20948_WHO_AM_I              0x00U
#define ICM20948_USER_CTRL             0x03U
#define ICM20948_LP_CONFIG             0x05U
#define ICM20948_PWR_MGMT_1            0x06U
#define ICM20948_PWR_MGMT_2            0x07U
#define ICM20948_ACCEL_XOUT_H          0x2DU

#define ICM20948_GYRO_SMPLRT_DIV       0x00U
#define ICM20948_GYRO_CONFIG_1         0x01U
#define ICM20948_ODR_ALIGN_EN          0x09U
#define ICM20948_ACCEL_SMPLRT_DIV_1    0x10U
#define ICM20948_ACCEL_SMPLRT_DIV_2    0x11U
#define ICM20948_ACCEL_CONFIG          0x14U

#define ICM20948_ID_VALUE              0xEAU
#define ICM20948_ADDR_AD0_LOW          0x68U
#define ICM20948_ADDR_AD0_HIGH         0x69U

#define ICM20948_ACC_SENS_2G           16384.0f
#define ICM20948_GYR_SENS_500DPS       65.5f
#define ICM20948_TEMP_SENS             333.87f
#define ICM20948_TEMP_OFFSET           21.0f
#define ICM20948_RAD_TO_DEG            57.2957795f

#define ICM20948_I2C_WAIT_SPINS        200000U
#define ICM20948_GYRO_CAL_SAMPLES      500U
#define ICM20948_GYRO_CONFIG_500DPS    0x1BU
#define ICM20948_GYRO_DEADBAND_DPS     0.05f
#define ICM20948_STILL_GYRO_DPS        0.60f
#define ICM20948_STILL_ACCEL_MIN_G2    0.90f
#define ICM20948_STILL_ACCEL_MAX_G2    1.10f
#define ICM20948_STILL_COUNT_MIN       50U
#define ICM20948_BIAS_TRACK_TAU_S      2.0f
#define ICM20948_BIAS_TRACK_MAX_GAIN   0.02f

static uint8_t g_icm20948_addr7 = ICM20948_ADDR_AD0_HIGH;
static uint8_t g_icm20948_current_bank = 0xFFU;
static bool g_icm20948_ready;

static ICM20948_Data_t g_icm20948_data;
static ICM20948_Angle_t g_icm20948_angle;

static float g_icm20948_gyro_offset_x;
static float g_icm20948_gyro_offset_y;
static float g_icm20948_gyro_offset_z;
static float g_icm20948_roll_zero;
static float g_icm20948_pitch_zero;
static uint16_t g_icm20948_still_count;

static void icm20948_i2c_init(void);
static void icm20948_i2c_recover_bus(void);
static uint8_t icm20948_i2c_wait_idle(void);
static uint8_t icm20948_i2c_wait_not_busy(void);
static uint8_t icm20948_i2c_write(uint8_t addr7, const uint8_t *data,
    uint16_t length);
static uint8_t icm20948_i2c_read_reg(uint8_t addr7, uint8_t reg,
    uint8_t *data, uint16_t length);
static void icm20948_i2c_start_reg_read(uint8_t addr7, uint16_t length);
static uint8_t icm20948_switch_bank(uint8_t bank);
static float icm20948_wrap_angle(float angle);
static float icm20948_deadband(float value);
static void icm20948_calc_accel_angle(const ICM20948_Data_t *data,
    float *roll, float *pitch);
static bool icm20948_is_stationary(const ICM20948_Data_t *data);
static void icm20948_track_gyro_bias(const ICM20948_Data_t *data, float dt);

static void icm20948_i2c_init(void)
{
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST,
        DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST,
        DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_enableController(I2C_0_INST);
}

static void icm20948_i2c_recover_bus(void)
{
    SYSCFG_DL_I2C_0_init();
    icm20948_i2c_init();
}

static uint8_t icm20948_i2c_wait_idle(void)
{
    uint32_t spins = ICM20948_I2C_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return 2U;
        }
        if (spins-- == 0U) {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t icm20948_i2c_wait_not_busy(void)
{
    uint32_t spins = ICM20948_I2C_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (spins-- == 0U) {
            return 1U;
        }
    }

    if ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return 2U;
    }

    return 0U;
}

static uint8_t icm20948_i2c_write(uint8_t addr7, const uint8_t *data,
    uint16_t length)
{
    uint8_t retry;

    if ((data == NULL) || (length == 0U) || (length > 8U)) {
        return 1U;
    }

    icm20948_i2c_init();

    for (retry = 0U; retry < 2U; retry++) {
        if (icm20948_i2c_wait_idle() != 0U) {
            icm20948_i2c_recover_bus();
            continue;
        }

        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_flushControllerRXFIFO(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, (uint8_t *) data,
                length) != length) {
            return 2U;
        }

        DL_I2C_startControllerTransfer(I2C_0_INST, addr7,
            DL_I2C_CONTROLLER_DIRECTION_TX, length);
        delay_cpu_cycles(16U);

        if (icm20948_i2c_wait_not_busy() == 0U) {
            return 0U;
        }

        icm20948_i2c_recover_bus();
    }

    return 3U;
}

static uint8_t icm20948_i2c_read_reg(uint8_t addr7, uint8_t reg,
    uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint8_t retry;

    if ((data == NULL) || (length == 0U)) {
        return 1U;
    }

    icm20948_i2c_init();

    for (retry = 0U; retry < 2U; retry++) {
        if (icm20948_i2c_wait_idle() != 0U) {
            icm20948_i2c_recover_bus();
            continue;
        }

        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_flushControllerRXFIFO(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1U) != 1U) {
            return 2U;
        }

        icm20948_i2c_start_reg_read(addr7, length);
        delay_cpu_cycles(16U);

        for (i = 0U; i < length; i++) {
            uint32_t spins = ICM20948_I2C_WAIT_SPINS;

            while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
                if ((DL_I2C_getControllerStatus(I2C_0_INST) &
                        DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                    break;
                }
                if (spins-- == 0U) {
                    break;
                }
            }

            if (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
                break;
            }

            data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
        }

        if ((i == length) && (icm20948_i2c_wait_not_busy() == 0U)) {
            return 0U;
        }

        icm20948_i2c_recover_bus();
    }

    return 3U;
}

static void icm20948_i2c_start_reg_read(uint8_t addr7, uint16_t length)
{
    DL_Common_updateReg(&I2C_0_INST->MASTER.MSA,
        (((uint32_t) addr7 << I2C_MSA_SADDR_OFS) |
            (uint32_t) DL_I2C_CONTROLLER_DIRECTION_RX),
        (I2C_MSA_SADDR_MASK | I2C_MSA_DIR_MASK));

    DL_Common_updateReg(&I2C_0_INST->MASTER.MCTR,
        (((uint32_t) length << I2C_MCTR_MBLEN_OFS) |
            I2C_MCTR_BURSTRUN_ENABLE |
            I2C_MCTR_START_ENABLE |
            I2C_MCTR_RD_ON_TXEMPTY_ENABLE |
            I2C_MCTR_STOP_ENABLE),
        (I2C_MCTR_MBLEN_MASK |
            I2C_MCTR_BURSTRUN_MASK |
            I2C_MCTR_START_MASK |
            I2C_MCTR_STOP_MASK |
            I2C_MCTR_RD_ON_TXEMPTY_MASK));
}

void ICM20948_SetDeviceAddr7bit(uint8_t addr7)
{
    g_icm20948_addr7 = (uint8_t) (addr7 & 0x7FU);
    g_icm20948_current_bank = 0xFFU;
}

uint8_t ICM20948_GetDeviceAddr7bit(void)
{
    return g_icm20948_addr7;
}

bool ICM20948_IsReady(void)
{
    return g_icm20948_ready;
}

uint8_t ICM20948_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t packet[2];

    packet[0] = reg;
    packet[1] = data;

    return icm20948_i2c_write(g_icm20948_addr7, packet, 2U);
}

uint8_t ICM20948_ReadReg(uint8_t reg, uint8_t *data)
{
    return ICM20948_ReadRegs(reg, data, 1U);
}

uint8_t ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return icm20948_i2c_read_reg(g_icm20948_addr7, reg, buf, len);
}

static uint8_t icm20948_switch_bank(uint8_t bank)
{
    if (g_icm20948_current_bank == bank) {
        return 0U;
    }

    if (ICM20948_WriteReg(ICM20948_REG_BANK_SEL,
            (uint8_t) (bank << 4)) != 0U) {
        return 1U;
    }

    g_icm20948_current_bank = bank;
    delay_ms(1U);

    return 0U;
}

uint8_t ICM20948_Probe(uint8_t addr7, uint8_t *whoami)
{
    uint8_t old_addr = g_icm20948_addr7;
    uint8_t id = 0xFFU;
    uint8_t ret;

    ICM20948_SetDeviceAddr7bit(addr7);
    g_icm20948_current_bank = 0xFFU;

    ret = icm20948_switch_bank(0U);
    if (ret == 0U) {
        ret = ICM20948_ReadReg(ICM20948_WHO_AM_I, &id);
    }

    ICM20948_SetDeviceAddr7bit(old_addr);

    if (whoami != NULL) {
        *whoami = id;
    }

    return ret;
}

uint8_t ICM20948_WhoAmI(void)
{
    uint8_t id = 0xFFU;

    if (icm20948_switch_bank(0U) != 0U) {
        return 0xFFU;
    }
    if (ICM20948_ReadReg(ICM20948_WHO_AM_I, &id) != 0U) {
        return 0xFFU;
    }

    return id;
}

uint8_t ICM20948_Init(void)
{
    uint8_t id = 0xFFU;
    uint8_t reg = 0U;
    uint8_t ret;

    g_icm20948_ready = false;
    g_icm20948_current_bank = 0xFFU;
    icm20948_i2c_recover_bus();
    delay_ms(20U);

    ret = ICM20948_Probe(ICM20948_ADDR_AD0_HIGH, &id);
    if ((ret == 0U) && (id == ICM20948_ID_VALUE)) {
        ICM20948_SetDeviceAddr7bit(ICM20948_ADDR_AD0_HIGH);
    } else {
        ret = ICM20948_Probe(ICM20948_ADDR_AD0_LOW, &id);
        if ((ret == 0U) && (id == ICM20948_ID_VALUE)) {
            ICM20948_SetDeviceAddr7bit(ICM20948_ADDR_AD0_LOW);
        } else {
            return 1U;
        }
    }

    g_icm20948_current_bank = 0xFFU;
    if (icm20948_switch_bank(0U) != 0U) return 2U;
    if (ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x80U) != 0U) return 3U;
    delay_ms(50U);

    g_icm20948_current_bank = 0xFFU;
    if (icm20948_switch_bank(0U) != 0U) return 4U;
    if (ICM20948_ReadReg(ICM20948_PWR_MGMT_1, &reg) != 0U) return 5U;

    reg &= (uint8_t) (~0x40U);
    reg &= (uint8_t) (~0x07U);
    reg |= 0x01U;
    if (ICM20948_WriteReg(ICM20948_PWR_MGMT_1, reg) != 0U) return 6U;
    delay_ms(10U);

    if (ICM20948_WriteReg(ICM20948_PWR_MGMT_2, 0x00U) != 0U) return 7U;
    if (ICM20948_WriteReg(ICM20948_LP_CONFIG, 0x00U) != 0U) return 8U;

    if (icm20948_switch_bank(2U) != 0U) return 10U;
    if (ICM20948_WriteReg(ICM20948_ODR_ALIGN_EN, 0x01U) != 0U) return 11U;

    if (ICM20948_WriteReg(ICM20948_GYRO_SMPLRT_DIV, 0x04U) != 0U) return 12U;
    if (ICM20948_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00U) != 0U) {
        return 13U;
    }
    if (ICM20948_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 0x04U) != 0U) {
        return 14U;
    }

    if (ICM20948_WriteReg(ICM20948_GYRO_CONFIG_1,
            ICM20948_GYRO_CONFIG_500DPS) != 0U) return 15U;
    if (ICM20948_WriteReg(ICM20948_ACCEL_CONFIG, 0x00U) != 0U) return 16U;

    if (icm20948_switch_bank(0U) != 0U) return 17U;
    id = ICM20948_WhoAmI();
    if (id != ICM20948_ID_VALUE) return 18U;

    g_icm20948_gyro_offset_x = 0.0f;
    g_icm20948_gyro_offset_y = 0.0f;
    g_icm20948_gyro_offset_z = 0.0f;
    g_icm20948_roll_zero = 0.0f;
    g_icm20948_pitch_zero = 0.0f;
    g_icm20948_still_count = 0U;
    (void) ICM20948_GyroCalibrate(ICM20948_GYRO_CAL_SAMPLES);

    if (ICM20948_ReadData(&g_icm20948_data) != 0U) return 19U;
    icm20948_calc_accel_angle(&g_icm20948_data, &g_icm20948_roll_zero,
        &g_icm20948_pitch_zero);
    ICM20948_ResetAngle();
    g_icm20948_ready = true;

    return 0U;
}

uint8_t ICM20948_ReadRaw(ICM20948_Raw_t *raw)
{
    uint8_t buf[14];

    if (raw == NULL) {
        return 1U;
    }
    if (icm20948_switch_bank(0U) != 0U) {
        return 2U;
    }
    if (ICM20948_ReadRegs(ICM20948_ACCEL_XOUT_H, buf, 14U) != 0U) {
        return 3U;
    }

    raw->ax = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
    raw->ay = (int16_t) (((uint16_t) buf[2] << 8) | buf[3]);
    raw->az = (int16_t) (((uint16_t) buf[4] << 8) | buf[5]);
    raw->gx = (int16_t) (((uint16_t) buf[6] << 8) | buf[7]);
    raw->gy = (int16_t) (((uint16_t) buf[8] << 8) | buf[9]);
    raw->gz = (int16_t) (((uint16_t) buf[10] << 8) | buf[11]);
    raw->temp = (int16_t) (((uint16_t) buf[12] << 8) | buf[13]);

    return 0U;
}

uint8_t ICM20948_ReadData(ICM20948_Data_t *data)
{
    ICM20948_Raw_t raw;
    uint8_t ret;

    if (data == NULL) {
        return 1U;
    }

    ret = ICM20948_ReadRaw(&raw);
    if (ret != 0U) {
        return ret;
    }

    data->ax_g = (float) raw.ax / ICM20948_ACC_SENS_2G;
    data->ay_g = (float) raw.ay / ICM20948_ACC_SENS_2G;
    data->az_g = (float) raw.az / ICM20948_ACC_SENS_2G;

    data->gx_dps = (float) raw.gx / ICM20948_GYR_SENS_500DPS -
                   g_icm20948_gyro_offset_x;
    data->gy_dps = (float) raw.gy / ICM20948_GYR_SENS_500DPS -
                   g_icm20948_gyro_offset_y;
    data->gz_dps = (float) raw.gz / ICM20948_GYR_SENS_500DPS -
                   g_icm20948_gyro_offset_z;

    data->temp_c = (float) raw.temp / ICM20948_TEMP_SENS +
                   ICM20948_TEMP_OFFSET;

    return 0U;
}

uint8_t ICM20948_GyroCalibrate(uint16_t sample_num)
{
    uint16_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    ICM20948_Raw_t raw;

    if (sample_num == 0U) {
        return 1U;
    }

    for (i = 0U; i < sample_num; i++) {
        if (ICM20948_ReadRaw(&raw) != 0U) {
            return 2U;
        }

        sum_x += (float) raw.gx / ICM20948_GYR_SENS_500DPS;
        sum_y += (float) raw.gy / ICM20948_GYR_SENS_500DPS;
        sum_z += (float) raw.gz / ICM20948_GYR_SENS_500DPS;

        delay_ms(2U);
    }

    g_icm20948_gyro_offset_x = sum_x / (float) sample_num;
    g_icm20948_gyro_offset_y = sum_y / (float) sample_num;
    g_icm20948_gyro_offset_z = sum_z / (float) sample_num;

    return 0U;
}

void ICM20948_ResetAngle(void)
{
    g_icm20948_angle.roll = 0.0f;
    g_icm20948_angle.pitch = 0.0f;
    g_icm20948_angle.yaw = 0.0f;
    g_icm20948_still_count = 0U;
}

void ICM20948_UpdateAngle(float dt)
{
    float roll_acc;
    float pitch_acc;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float yaw_gyro;
    const float alpha = 0.70f;

    if ((dt <= 0.0f) || (ICM20948_ReadData(&g_icm20948_data) != 0U)) {
        return;
    }

    gx_dps = g_icm20948_data.gx_dps;
    gy_dps = g_icm20948_data.gy_dps;
    gz_dps = g_icm20948_data.gz_dps;

    if (icm20948_is_stationary(&g_icm20948_data)) {
        if (g_icm20948_still_count < ICM20948_STILL_COUNT_MIN) {
            g_icm20948_still_count++;
        } else {
            icm20948_track_gyro_bias(&g_icm20948_data, dt);
            gx_dps = 0.0f;
            gy_dps = 0.0f;
            gz_dps = 0.0f;
        }
    } else {
        g_icm20948_still_count = 0U;
    }

    gx_dps = icm20948_deadband(gx_dps);
    gy_dps = icm20948_deadband(gy_dps);
    gz_dps = icm20948_deadband(gz_dps);

    icm20948_calc_accel_angle(&g_icm20948_data, &roll_acc, &pitch_acc);
    roll_acc = icm20948_wrap_angle(roll_acc - g_icm20948_roll_zero);
    pitch_acc = icm20948_wrap_angle(pitch_acc - g_icm20948_pitch_zero);

    g_icm20948_angle.roll =
        alpha * (g_icm20948_angle.roll + gx_dps * dt) +
        (1.0f - alpha) * roll_acc;

    g_icm20948_angle.pitch =
        alpha * (g_icm20948_angle.pitch + gy_dps * dt) +
        (1.0f - alpha) * pitch_acc;

    yaw_gyro = icm20948_wrap_angle(g_icm20948_angle.yaw +
        gz_dps * dt);

    g_icm20948_angle.yaw = yaw_gyro;
}

ICM20948_Angle_t ICM20948_GetAngle(void)
{
    return g_icm20948_angle;
}

ICM20948_Data_t ICM20948_GetData(void)
{
    return g_icm20948_data;
}

static bool icm20948_is_stationary(const ICM20948_Data_t *data)
{
    float accel_g2;

    if (data == NULL) {
        return false;
    }

    if ((fabsf(data->gx_dps) > ICM20948_STILL_GYRO_DPS) ||
        (fabsf(data->gy_dps) > ICM20948_STILL_GYRO_DPS) ||
        (fabsf(data->gz_dps) > ICM20948_STILL_GYRO_DPS)) {
        return false;
    }

    accel_g2 = data->ax_g * data->ax_g + data->ay_g * data->ay_g +
               data->az_g * data->az_g;

    return ((accel_g2 >=
                (ICM20948_STILL_ACCEL_MIN_G2 * ICM20948_STILL_ACCEL_MIN_G2)) &&
            (accel_g2 <=
                (ICM20948_STILL_ACCEL_MAX_G2 * ICM20948_STILL_ACCEL_MAX_G2)));
}

static float icm20948_deadband(float value)
{
    if (fabsf(value) < ICM20948_GYRO_DEADBAND_DPS) {
        return 0.0f;
    }

    return value;
}

static void icm20948_calc_accel_angle(const ICM20948_Data_t *data,
    float *roll, float *pitch)
{
    if ((data == NULL) || (roll == NULL) || (pitch == NULL)) {
        return;
    }

    *roll = atan2f(data->ay_g, data->az_g) * ICM20948_RAD_TO_DEG;
    *pitch = atan2f(-data->ax_g,
        sqrtf(data->ay_g * data->ay_g + data->az_g * data->az_g)) *
        ICM20948_RAD_TO_DEG;
}

static void icm20948_track_gyro_bias(const ICM20948_Data_t *data, float dt)
{
    float gain;

    if ((data == NULL) || (dt <= 0.0f)) {
        return;
    }

    gain = dt / ICM20948_BIAS_TRACK_TAU_S;
    if (gain > ICM20948_BIAS_TRACK_MAX_GAIN) {
        gain = ICM20948_BIAS_TRACK_MAX_GAIN;
    }

    g_icm20948_gyro_offset_x += data->gx_dps * gain;
    g_icm20948_gyro_offset_y += data->gy_dps * gain;
    g_icm20948_gyro_offset_z += data->gz_dps * gain;
}

static float icm20948_wrap_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}
