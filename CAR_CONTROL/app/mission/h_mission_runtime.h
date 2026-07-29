#ifndef APP_MISSION_H_MISSION_RUNTIME_H
#define APP_MISSION_H_MISSION_RUNTIME_H

#include "h_mission.h"
#include "h_route_events.h"

#include <stdbool.h>
#include <stdint.h>

#define H_MISSION_RUNTIME_PRECISION_SPEED_PPS 700.0f
#define H_MISSION_RUNTIME_PRECISION_TIMEOUT_MS 4000U
#define H_MISSION_ROUTE_FINISH_REARM_MS       300U
/* Tuned after measuring the first physical stop at the finish A marker. */
#define H_MISSION_ROUTE_PRECISION_DELTA_COUNT    0

typedef struct {
    h_mission_snapshot_t mission;
    h_route_snapshot_t route;
    bool route_ready;
    bool line_owned;
    bool precision_owned;
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
