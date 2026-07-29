#ifndef APP_MISSION_H_ROUTE_EVENTS_H
#define APP_MISSION_H_ROUTE_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t leave_a_min_count;
    int32_t initial_a_max_count;
    int32_t b_passage_count;
    int32_t finish_arm_count;
    int32_t precision_stop_delta_count;
    uint8_t marker_active_min;
    uint16_t marker_confirm_ms;
    uint16_t marker_release_ms;
} h_route_config_t;

typedef struct {
    uint32_t now_ms;
    int32_t odometry_count;
    uint8_t line_active_count;
    bool odometry_ready;
    bool line_sensor_ready;
} h_route_input_t;

typedef struct {
    bool configuration_valid;
    bool input_valid;
    bool running;
    bool marker_wide;
    bool initial_a_seen;
    bool left_start_a;
    bool b_passed;
    bool finish_a_passed;
    bool left_start_a_event;
    bool b_passed_event;
    bool finish_a_passed_event;
    int32_t start_count;
    int32_t progress_count;
    uint32_t marker_count;
} h_route_snapshot_t;

typedef struct {
    h_route_config_t config;
    h_route_snapshot_t snapshot;
    bool raw_marker_wide;
    bool marker_rise_event;
    uint32_t raw_marker_changed_ms;
} h_route_events_t;

bool HRouteEvents_ConfigIsValid(const h_route_config_t *config);
void HRouteEvents_Init(h_route_events_t *events,
    const h_route_config_t *config);
bool HRouteEvents_SetConfig(h_route_events_t *events,
    const h_route_config_t *config);
bool HRouteEvents_Start(h_route_events_t *events,
    uint32_t now_ms, int32_t start_count);
void HRouteEvents_Stop(h_route_events_t *events);
void HRouteEvents_Update(h_route_events_t *events,
    const h_route_input_t *input);
bool HRouteEvents_GetSnapshot(const h_route_events_t *events,
    h_route_snapshot_t *snapshot);

#endif
