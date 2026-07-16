#include "i2c0_polling.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define I2C0_POLLING_WAIT_SPINS 200000U
#define I2C0_POLLING_RETRY_COUNT     2U
#define I2C0_POLLING_TX_FIFO_BYTES   8U

static i2c0_polling_snapshot_t g_snapshot;

static void i2c0_polling_configure_controller(void);
static i2c0_polling_result_t i2c0_polling_wait_idle(void);
static i2c0_polling_result_t i2c0_polling_wait_not_busy(void);
static void i2c0_polling_start_register_read(
    uint8_t address7, uint16_t length);

void I2C0Polling_Init(void)
{
    g_snapshot.transaction_count = 0U;
    g_snapshot.recovery_count = 0U;
    g_snapshot.last_result = I2C0_POLLING_OK;
    g_snapshot.initialized = true;
    i2c0_polling_configure_controller();
}

void I2C0Polling_Recover(void)
{
    SYSCFG_DL_I2C_0_init();
    i2c0_polling_configure_controller();
    g_snapshot.recovery_count++;
    g_snapshot.initialized = true;
}

i2c0_polling_result_t I2C0Polling_Write(
    uint8_t address7, const uint8_t *data, uint16_t length)
{
    uint8_t retry;
    i2c0_polling_result_t result = I2C0_POLLING_TIMEOUT;

    if ((data == NULL) || (length == 0U) ||
        (length > I2C0_POLLING_TX_FIFO_BYTES) ||
        (address7 > 0x7FU)) {
        g_snapshot.last_result = I2C0_POLLING_BAD_ARGUMENT;
        return g_snapshot.last_result;
    }
    if (!g_snapshot.initialized) {
        I2C0Polling_Init();
    }
    g_snapshot.transaction_count++;

    for (retry = 0U; retry < I2C0_POLLING_RETRY_COUNT; retry++) {
        result = i2c0_polling_wait_idle();
        if (result != I2C0_POLLING_OK) {
            I2C0Polling_Recover();
            continue;
        }

        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_flushControllerRXFIFO(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if (DL_I2C_fillControllerTXFIFO(I2C_0_INST,
                (uint8_t *) data, length) != length) {
            g_snapshot.last_result = I2C0_POLLING_FIFO_ERROR;
            return g_snapshot.last_result;
        }

        DL_I2C_startControllerTransfer(I2C_0_INST, address7,
            DL_I2C_CONTROLLER_DIRECTION_TX, length);
        delay_cpu_cycles(16U);
        result = i2c0_polling_wait_not_busy();
        if (result == I2C0_POLLING_OK) {
            g_snapshot.last_result = I2C0_POLLING_OK;
            return I2C0_POLLING_OK;
        }
        I2C0Polling_Recover();
    }

    g_snapshot.last_result = result;
    return result;
}

i2c0_polling_result_t I2C0Polling_ReadRegister(
    uint8_t address7, uint8_t reg, uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint8_t retry;
    i2c0_polling_result_t result = I2C0_POLLING_TIMEOUT;

    if ((data == NULL) || (length == 0U) || (address7 > 0x7FU)) {
        g_snapshot.last_result = I2C0_POLLING_BAD_ARGUMENT;
        return g_snapshot.last_result;
    }
    if (!g_snapshot.initialized) {
        I2C0Polling_Init();
    }
    g_snapshot.transaction_count++;

    for (retry = 0U; retry < I2C0_POLLING_RETRY_COUNT; retry++) {
        result = i2c0_polling_wait_idle();
        if (result != I2C0_POLLING_OK) {
            I2C0Polling_Recover();
            continue;
        }

        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_flushControllerRXFIFO(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if (DL_I2C_fillControllerTXFIFO(
                I2C_0_INST, &reg, 1U) != 1U) {
            g_snapshot.last_result = I2C0_POLLING_FIFO_ERROR;
            return g_snapshot.last_result;
        }

        i2c0_polling_start_register_read(address7, length);
        delay_cpu_cycles(16U);

        for (index = 0U; index < length; index++) {
            uint32_t spins = I2C0_POLLING_WAIT_SPINS;

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
            data[index] = DL_I2C_receiveControllerData(I2C_0_INST);
        }

        result = i2c0_polling_wait_not_busy();
        if ((index == length) && (result == I2C0_POLLING_OK)) {
            g_snapshot.last_result = I2C0_POLLING_OK;
            return I2C0_POLLING_OK;
        }
        if (index != length) {
            result = I2C0_POLLING_TIMEOUT;
        }
        I2C0Polling_Recover();
    }

    g_snapshot.last_result = result;
    return result;
}

bool I2C0Polling_GetSnapshot(i2c0_polling_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static void i2c0_polling_configure_controller(void)
{
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_setControllerTXFIFOThreshold(
        I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(
        I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
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

static i2c0_polling_result_t i2c0_polling_wait_idle(void)
{
    uint32_t spins = I2C0_POLLING_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return I2C0_POLLING_BUS_ERROR;
        }
        if (spins-- == 0U) {
            return I2C0_POLLING_TIMEOUT;
        }
    }
    return I2C0_POLLING_OK;
}

static i2c0_polling_result_t i2c0_polling_wait_not_busy(void)
{
    uint32_t spins = I2C0_POLLING_WAIT_SPINS;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (spins-- == 0U) {
            return I2C0_POLLING_TIMEOUT;
        }
    }
    if ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return I2C0_POLLING_BUS_ERROR;
    }
    return I2C0_POLLING_OK;
}

static void i2c0_polling_start_register_read(
    uint8_t address7, uint16_t length)
{
    DL_Common_updateReg(&I2C_0_INST->MASTER.MSA,
        (((uint32_t) address7 << I2C_MSA_SADDR_OFS) |
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
