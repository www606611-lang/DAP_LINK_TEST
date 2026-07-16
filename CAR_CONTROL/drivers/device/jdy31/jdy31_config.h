#ifndef DRIVERS_DEVICE_JDY31_CONFIG_H
#define DRIVERS_DEVICE_JDY31_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    JDY31_CONFIG_DISABLED = 0,
    JDY31_CONFIG_WAIT_START,
    JDY31_CONFIG_WAIT_DISC,
    JDY31_CONFIG_PROBE_9600,
    JDY31_CONFIG_PROBE_115200,
    JDY31_CONFIG_QUERY_BAUD,
    JDY31_CONFIG_SET_BAUD,
    JDY31_CONFIG_VERIFY_DELAY,
    JDY31_CONFIG_VERIFY_BAUD,
    JDY31_CONFIG_POWER_CYCLE,
    JDY31_CONFIG_SUCCESS,
    JDY31_CONFIG_FAILED
} jdy31_config_state_t;

typedef struct {
    jdy31_config_state_t state;
    bool exclusive;
    bool success;
    uint32_t uart_baud;
    uint32_t command_attempts;
    int32_t reported_baud_code;
    char last_response[32];
} jdy31_config_snapshot_t;

void JDY31_ConfigInit(uint32_t now_ms, bool configure_on_boot);
void JDY31_ConfigTask(uint32_t now_ms);
bool JDY31_ConfigIsExclusive(void);
bool JDY31_ConfigGetSnapshot(jdy31_config_snapshot_t *snapshot);
const char *JDY31_ConfigGetStateText(void);

#endif
