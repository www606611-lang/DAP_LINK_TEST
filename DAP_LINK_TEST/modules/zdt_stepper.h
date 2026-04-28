#ifndef MODULES_ZDT_STEPPER_H
#define MODULES_ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_STEPPER_MAX_RPM        (5000U)
#define ZDT_STEPPER_ADDR_BROADCAST (0U)

typedef enum {
    ZDT_STEPPER_1 = 0,
    ZDT_STEPPER_2,
    ZDT_STEPPER_COUNT
} zdt_stepper_id_t;

typedef enum {
    ZDT_STEPPER_DIR_CW = 0,
    ZDT_STEPPER_DIR_CCW = 1
} zdt_stepper_dir_t;

typedef enum {
    ZDT_STEPPER_PARAM_VERSION = 0,
    ZDT_STEPPER_PARAM_RL,
    ZDT_STEPPER_PARAM_PID,
    ZDT_STEPPER_PARAM_VBUS,
    ZDT_STEPPER_PARAM_PHASE_CURRENT,
    ZDT_STEPPER_PARAM_ENCODER,
    ZDT_STEPPER_PARAM_TARGET_POSITION,
    ZDT_STEPPER_PARAM_SPEED,
    ZDT_STEPPER_PARAM_POSITION,
    ZDT_STEPPER_PARAM_POSITION_ERROR,
    ZDT_STEPPER_PARAM_FLAGS,
    ZDT_STEPPER_PARAM_CONFIG,
    ZDT_STEPPER_PARAM_STATE,
    ZDT_STEPPER_PARAM_ORIGIN
} zdt_stepper_param_t;

typedef struct {
    uint32_t ext_id;
    uint8_t address;
    uint8_t packet;
    uint8_t dlc;
    uint8_t data[8];
} zdt_stepper_can_frame_t;

void ZdtStepper_Init(void);

bool ZdtStepper_SetAddress(zdt_stepper_id_t motor, uint8_t address);
uint8_t ZdtStepper_GetAddress(zdt_stepper_id_t motor);

void ZdtStepper_SetInverted(zdt_stepper_id_t motor, bool inverted);
bool ZdtStepper_IsInverted(zdt_stepper_id_t motor);

bool ZdtStepper_Enable(zdt_stepper_id_t motor, bool enable);
bool ZdtStepper_EnableAll(bool enable);

bool ZdtStepper_SetSpeed(zdt_stepper_id_t motor, int16_t rpm, uint8_t acc);
bool ZdtStepper_SetSpeedSync(
    zdt_stepper_id_t motor, int16_t rpm, uint8_t acc, bool sync);
int16_t ZdtStepper_GetTargetSpeedRpm(zdt_stepper_id_t motor);
int32_t ZdtStepper_GetTargetPulseCount(zdt_stepper_id_t motor);

bool ZdtStepper_MoveRelative(
    zdt_stepper_id_t motor, int32_t pulses, uint16_t rpm, uint8_t acc,
    bool sync);
bool ZdtStepper_MoveAbsolute(
    zdt_stepper_id_t motor, int32_t position, uint16_t rpm, uint8_t acc,
    bool sync);
bool ZdtStepper_StartSync(void);

bool ZdtStepper_Stop(zdt_stepper_id_t motor, bool sync);
bool ZdtStepper_StopAll(void);

bool ZdtStepper_ResetPosition(zdt_stepper_id_t motor);
bool ZdtStepper_ClearStall(zdt_stepper_id_t motor);
bool ZdtStepper_RequestParam(
    zdt_stepper_id_t motor, zdt_stepper_param_t param);

bool ZdtStepper_SetOriginHere(zdt_stepper_id_t motor, bool save);
bool ZdtStepper_TriggerOrigin(
    zdt_stepper_id_t motor, uint8_t mode, bool sync);
bool ZdtStepper_AbortOrigin(zdt_stepper_id_t motor);

bool ZdtStepper_ReadFrame(zdt_stepper_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
