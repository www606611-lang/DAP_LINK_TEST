#include "line_sensor_i2c.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define LINE_SENSOR_I2C_ADDR7          0x12U
#define LINE_SENSOR_REG_CONTROL        0x01U
#define LINE_SENSOR_REG_DATA           0x30U
#define LINE_SENSOR_I2C_WAIT_SPINS     200000U

static uint8_t g_line_sensor_last_error;

static void line_sensor_i2c_configure(void);
static void line_sensor_i2c_recover_bus(void);
static bool line_sensor_i2c_wait_idle(void);
static bool line_sensor_i2c_wait_not_busy(void);
static bool line_sensor_i2c_write(uint8_t reg, uint8_t data);
static bool line_sensor_i2c_read_reg(uint8_t reg, uint8_t *data);
static void line_sensor_i2c_start_reg_read(uint16_t length);

void LineSensorI2C_Init(void)
{
    g_line_sensor_last_error = 0U;
    line_sensor_i2c_configure();

    /*
     * 参考工程里这颗 8 路巡线模块上电后会先进入一次校准态，
     * 再退出校准。这里在初始化时补上同样的动作，避免读数一直停在默认值。
     */
    delay_ms(200U);
    (void) LineSensorI2C_SetCalibration(true);
    delay_ms(200U);
    (void) LineSensorI2C_SetCalibration(false);
    delay_ms(200U);
}

bool LineSensorI2C_ReadRaw(uint8_t *raw)
{
    if (raw == NULL) {
        g_line_sensor_last_error = 1U;
        return false;
    }

    return line_sensor_i2c_read_reg(LINE_SENSOR_REG_DATA, raw);
}

bool LineSensorI2C_ReadState(line_sensor_i2c_state_t *state)
{
    uint8_t raw;
    uint8_t i;

    if (state == NULL) {
        g_line_sensor_last_error = 1U;
        return false;
    }

    if (!LineSensorI2C_ReadRaw(&raw)) {
        return false;
    }

    state->raw = raw;
    state->active_mask = 0U;
    state->active_count = 0U;

    for (i = 0U; i < LINE_SENSOR_I2C_CHANNEL_COUNT; i++) {
        uint8_t bit = (uint8_t) (0x80U >> i);
        bool active = ((raw & bit) == 0U);

        state->channel[i] = active;
        if (active) {
            state->active_mask |= bit;
            state->active_count++;
        }
    }

    return true;
}

bool LineSensorI2C_SetCalibration(bool enabled)
{
    return line_sensor_i2c_write(
        LINE_SENSOR_REG_CONTROL, enabled ? 1U : 0U);
}

uint8_t LineSensorI2C_GetLastError(void)
{
    return g_line_sensor_last_error;
}

static void line_sensor_i2c_configure(void)
{
    DL_I2C_resetControllerTransfer(I2C_1_INST);
    DL_I2C_setTimerPeriod(I2C_1_INST, 9U);
    DL_I2C_setControllerTXFIFOThreshold(I2C_1_INST,
        DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_1_INST,
        DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_1_INST);
    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(I2C_1_INST);
    DL_I2C_clearInterruptStatus(I2C_1_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_enableController(I2C_1_INST);
}

static void line_sensor_i2c_recover_bus(void)
{
    SYSCFG_DL_I2C_1_init();
    line_sensor_i2c_configure();
}

static bool line_sensor_i2c_wait_idle(void)
{
    uint32_t spins = LINE_SENSOR_I2C_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_1_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((DL_I2C_getControllerStatus(I2C_1_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_line_sensor_last_error = 2U;
            return false;
        }
        if (spins-- == 0U) {
            g_line_sensor_last_error = 3U;
            return false;
        }
    }

    return true;
}

static bool line_sensor_i2c_wait_not_busy(void)
{
    uint32_t spins = LINE_SENSOR_I2C_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_1_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (spins-- == 0U) {
            g_line_sensor_last_error = 4U;
            return false;
        }
    }

    if ((DL_I2C_getControllerStatus(I2C_1_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        g_line_sensor_last_error = 5U;
        return false;
    }

    return true;
}

static bool line_sensor_i2c_write(uint8_t reg, uint8_t data)
{
    uint8_t packet[2];

    packet[0] = reg;
    packet[1] = data;
    line_sensor_i2c_configure();

    if (!line_sensor_i2c_wait_idle()) {
        return false;
    }

    DL_I2C_resetControllerTransfer(I2C_1_INST);
    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(I2C_1_INST);
    DL_I2C_clearInterruptStatus(I2C_1_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

    if (DL_I2C_fillControllerTXFIFO(I2C_1_INST, packet, 2U) != 2U) {
        g_line_sensor_last_error = 6U;
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_1_INST, LINE_SENSOR_I2C_ADDR7,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cpu_cycles(16U);

    if (!line_sensor_i2c_wait_not_busy()) {
        return false;
    }

    g_line_sensor_last_error = 0U;
    return true;
}

static bool line_sensor_i2c_read_reg(uint8_t reg, uint8_t *data)
{
    uint8_t retry;

    if (data == NULL) {
        g_line_sensor_last_error = 1U;
        return false;
    }

    line_sensor_i2c_configure();

    for (retry = 0U; retry < 2U; retry++) {
        uint32_t spins = LINE_SENSOR_I2C_WAIT_SPINS;

        if (!line_sensor_i2c_wait_idle()) {
            line_sensor_i2c_recover_bus();
            continue;
        }

        DL_I2C_resetControllerTransfer(I2C_1_INST);
        DL_I2C_flushControllerTXFIFO(I2C_1_INST);
        DL_I2C_flushControllerRXFIFO(I2C_1_INST);
        DL_I2C_clearInterruptStatus(I2C_1_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if (DL_I2C_fillControllerTXFIFO(I2C_1_INST, &reg, 1U) != 1U) {
            g_line_sensor_last_error = 6U;
            return false;
        }

        line_sensor_i2c_start_reg_read(1U);
        delay_cpu_cycles(16U);

        while (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {
            if ((DL_I2C_getControllerStatus(I2C_1_INST) &
                    DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                break;
            }
            if (spins-- == 0U) {
                break;
            }
        }

        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {
            *data = DL_I2C_receiveControllerData(I2C_1_INST);
            if (line_sensor_i2c_wait_not_busy()) {
                g_line_sensor_last_error = 0U;
                return true;
            }
        } else {
            g_line_sensor_last_error = 7U;
        }

        line_sensor_i2c_recover_bus();
    }

    return false;
}

static void line_sensor_i2c_start_reg_read(uint16_t length)
{
    DL_Common_updateReg(&I2C_1_INST->MASTER.MSA,
        (((uint32_t) LINE_SENSOR_I2C_ADDR7 << I2C_MSA_SADDR_OFS) |
            (uint32_t) DL_I2C_CONTROLLER_DIRECTION_RX),
        (I2C_MSA_SADDR_MASK | I2C_MSA_DIR_MASK));

    DL_Common_updateReg(&I2C_1_INST->MASTER.MCTR,
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
