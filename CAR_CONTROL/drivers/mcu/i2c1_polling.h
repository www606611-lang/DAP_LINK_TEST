#ifndef DRIVERS_MCU_I2C1_POLLING_H
#define DRIVERS_MCU_I2C1_POLLING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    I2C1_POLLING_OK = 0,
    I2C1_POLLING_BAD_ARGUMENT,
    I2C1_POLLING_FIFO_ERROR,
    I2C1_POLLING_BUS_ERROR,
    I2C1_POLLING_TIMEOUT
} i2c1_polling_result_t;

typedef struct {
    uint32_t transaction_count;
    uint32_t recovery_count;
    i2c1_polling_result_t last_result;
    bool initialized;
} i2c1_polling_snapshot_t;

void I2C1Polling_Init(void);
void I2C1Polling_Recover(void);
i2c1_polling_result_t I2C1Polling_Write(
    uint8_t address7, const uint8_t *data, uint16_t length);
i2c1_polling_result_t I2C1Polling_ReadRegister(
    uint8_t address7, uint8_t reg, uint8_t *data, uint16_t length);
bool I2C1Polling_GetSnapshot(i2c1_polling_snapshot_t *snapshot);

#endif
