#ifndef APP_MISSION_H_MISSION_RUNTIME_H
#define APP_MISSION_H_MISSION_RUNTIME_H

#include "h_mission.h"
#include "h_route_events.h"

#include <stdbool.h>
#include <stdint.h>

#define H_MISSION_RUNTIME_PRECISION_SPEED_PPS 700.0f
#define H_MISSION_RUNTIME_PRECISION_TIMEOUT_MS 4000U
#define H_MISSION_ROUTE_FINISH_REARM_MS       300U
#define H_MISSION_ROUTE_FINISH_ARM_COUNT      19000
/* Tuned after measuring the first physical stop at the finish A marker. */
#define H_MISSION_ROUTE_PRECISION_DELTA_COUNT    0

typedef enum {
    H_MISSION_SPEED_IDLE = 0,
    H_MISSION_SPEED_RAMP,
    H_MISSION_SPEED_CRUISE,
    H_MISSION_SPEED_STOPPING
} h_mission_speed_stage_t;

typedef struct {
    h_mission_snapshot_t mission;
    h_route_snapshot_t route;
    bool route_ready;
    bool line_owned;
    bool precision_owned;
    h_mission_speed_stage_t speed_stage;
    float requested_base_speed_pps;
    uint32_t executor_error_count;
} h_mission_runtime_snapshot_t;

void HMissionRuntime_Init(bool reset_locked, uint32_t now_ms);
void HMissionRuntime_Task(uint32_t now_ms);
bool HMissionRuntime_SetRouteConfig(const h_route_config_t *config);
void HMissionRuntime_RequestPrimary(void);
void HMissionRuntime_RequestStop(void);
bool HMissionRuntime_IsActive(void);
bool HMissionRuntime_GetSnapshot(
    h_mission_runtime_snapshot_t *snapshot);
const char *HMissionRuntime_GetStateText(void);
const char *HMissionRuntime_GetPhaseText(void);

#endif
