#ifndef APP_YAW_BRINGUP_TEST_H
#define APP_YAW_BRINGUP_TEST_H

#include "wheel_yaw_control.h"

#include <stdbool.h>
#include <stdint.h>

#define YAW_BRINGUP_OUTPUT_MIN 100U
#define YAW_BRINGUP_OUTPUT_MAX 1000U

typedef enum {
    YAW_BRINGUP_TEST_LOCKED = 0,
    YAW_BRINGUP_TEST_READY,
    YAW_BRINGUP_TEST_RUNNING,
    YAW_BRINGUP_TEST_COMPLETE,
    YAW_BRINGUP_TEST_ABORTED
} yaw_bringup_test_state_t;

typedef enum {
    YAW_BRINGUP_CONFIG_OK = 0,
    YAW_BRINGUP_CONFIG_BAD_ARGUMENT,
    YAW_BRINGUP_CONFIG_BAD_RANGE,
    YAW_BRINGUP_CONFIG_BUSY
} yaw_bringup_config_result_t;

typedef struct {
    wheel_yaw_control_config_t control;
    float target_yaw_deg;
    uint16_t output_limit_permille;
    uint32_t timeout_ms;
} yaw_bringup_config_t;

void YawBringupTest_Init(bool reset_locked);
void YawBringupTest_Task(uint32_t now_ms);
yaw_bringup_config_result_t YawBringupTest_SetConfig(
    const yaw_bringup_config_t *config);
bool YawBringupTest_GetConfig(yaw_bringup_config_t *config);
bool YawBringupTest_RequestStart(void);
void YawBringupTest_RequestStop(void);
yaw_bringup_test_state_t YawBringupTest_GetState(void);
const char *YawBringupTest_GetStateText(void);
uint32_t YawBringupTest_GetRunCount(void);

#endif
