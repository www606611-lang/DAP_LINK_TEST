#ifndef APP_POSITION_BRINGUP_TEST_H
#define APP_POSITION_BRINGUP_TEST_H

#include "wheel_position_control.h"

#include <stdbool.h>
#include <stdint.h>

#define POSITION_BRINGUP_TARGET_MAX_COUNTS 100000L
#define POSITION_BRINGUP_OUTPUT_MIN        100U
#define POSITION_BRINGUP_OUTPUT_MAX        1000U
#define POSITION_BRINGUP_STRESS_STEP_COUNT 24U

typedef enum {
    POSITION_BRINGUP_TEST_LOCKED = 0,
    POSITION_BRINGUP_TEST_READY,
    POSITION_BRINGUP_TEST_RUNNING,
    POSITION_BRINGUP_TEST_COMPLETE,
    POSITION_BRINGUP_TEST_ABORTED
} position_bringup_test_state_t;

typedef enum {
    POSITION_BRINGUP_PROFILE_SINGLE = 0,
    POSITION_BRINGUP_PROFILE_STRESS
} position_bringup_profile_t;

typedef enum {
    POSITION_BRINGUP_CONFIG_OK = 0,
    POSITION_BRINGUP_CONFIG_BAD_ARGUMENT,
    POSITION_BRINGUP_CONFIG_BAD_RANGE,
    POSITION_BRINGUP_CONFIG_BUSY
} position_bringup_config_result_t;

typedef struct {
    wheel_position_control_config_t control;
    int32_t target_counts;
    uint16_t output_limit_permille;
    uint32_t timeout_ms;
} position_bringup_config_t;

void PositionBringupTest_Init(bool reset_locked);
void PositionBringupTest_Task(uint32_t now_ms, bool press_event);
position_bringup_config_result_t PositionBringupTest_SetConfig(
    const position_bringup_config_t *config);
bool PositionBringupTest_GetConfig(position_bringup_config_t *config);
bool PositionBringupTest_RequestStart(void);
bool PositionBringupTest_RequestProfile(position_bringup_profile_t profile);
void PositionBringupTest_RequestStop(void);
position_bringup_test_state_t PositionBringupTest_GetState(void);
const char *PositionBringupTest_GetStateText(void);
position_bringup_profile_t PositionBringupTest_GetProfile(void);
const char *PositionBringupTest_GetProfileText(void);
uint8_t PositionBringupTest_GetCurrentStep(void);
uint8_t PositionBringupTest_GetStepCount(void);
uint8_t PositionBringupTest_GetCompletedMoveCount(void);
uint32_t PositionBringupTest_GetWorstFinalErrorCount(void);
uint32_t PositionBringupTest_GetLeftRecoveryCount(void);
uint32_t PositionBringupTest_GetRightRecoveryCount(void);
uint32_t PositionBringupTest_GetRunCount(void);

#endif
