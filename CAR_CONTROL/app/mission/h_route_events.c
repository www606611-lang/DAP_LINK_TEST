#include "h_route_events.h"

#include <stddef.h>

static void h_route_events_reset_run(h_route_events_t *events);
static void h_route_events_update_marker(h_route_events_t *events,
    const h_route_input_t *input);

bool HRouteEvents_ConfigIsValid(const h_route_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    return (config->precision_stop_delta_count >= -2000) &&
        (config->precision_stop_delta_count <= 2000) &&
        (config->finish_rearm_ms >= 100U) &&
        (config->finish_rearm_ms <= 5000U) &&
        (config->marker_active_min >= 4U) &&
        (config->marker_active_min <= 8U) &&
        (config->marker_confirm_ms > 0U) &&
        (config->marker_confirm_ms <= 200U) &&
        (config->marker_release_ms > 0U) &&
        (config->marker_release_ms <= 200U);
}

void HRouteEvents_Init(h_route_events_t *events,
    const h_route_config_t *config)
{
    if (events == NULL) {
        return;
    }

    if (config != NULL) {
        events->config = *config;
    } else {
        h_route_config_t empty = {0};
        events->config = empty;
    }
    events->snapshot.configuration_valid =
        HRouteEvents_ConfigIsValid(&events->config);
    h_route_events_reset_run(events);
}

bool HRouteEvents_SetConfig(h_route_events_t *events,
    const h_route_config_t *config)
{
    if ((events == NULL) || events->snapshot.running ||
        !HRouteEvents_ConfigIsValid(config)) {
        return false;
    }
    events->config = *config;
    events->snapshot.configuration_valid = true;
    h_route_events_reset_run(events);
    return true;
}

bool HRouteEvents_Start(h_route_events_t *events,
    uint32_t now_ms, int32_t start_count)
{
    if ((events == NULL) ||
        !HRouteEvents_ConfigIsValid(&events->config)) {
        return false;
    }

    h_route_events_reset_run(events);
    events->snapshot.configuration_valid = true;
    events->snapshot.running = true;
    events->snapshot.start_count = start_count;
    events->raw_marker_changed_ms = now_ms;
    return true;
}

void HRouteEvents_Stop(h_route_events_t *events)
{
    if (events != NULL) {
        events->snapshot.running = false;
    }
}

void HRouteEvents_Update(h_route_events_t *events,
    const h_route_input_t *input)
{
    if ((events == NULL) || (input == NULL)) {
        return;
    }

    events->snapshot.left_start_a_event = false;
    events->snapshot.finish_a_passed_event = false;
    events->marker_rise_event = false;
    events->snapshot.input_valid = input->odometry_ready &&
        input->line_sensor_ready;
    if (!events->snapshot.configuration_valid) {
        return;
    }
    if (!events->snapshot.input_valid) {
        events->raw_marker_changed_ms = input->now_ms;
        return;
    }

    h_route_events_update_marker(events, input);
    if (!events->snapshot.running) {
        events->snapshot.progress_count = 0;
        return;
    }

    events->snapshot.progress_count =
        input->odometry_count - events->snapshot.start_count;

    if (!events->snapshot.initial_a_seen &&
        events->snapshot.marker_wide) {
        events->snapshot.initial_a_seen = true;
    }

    if (events->snapshot.initial_a_seen &&
        !events->snapshot.left_start_a &&
        !events->snapshot.marker_wide) {
        events->snapshot.left_start_a = true;
        events->snapshot.left_start_a_event = true;
        events->left_start_ms = input->now_ms;
    }

    if (events->snapshot.left_start_a &&
        !events->snapshot.finish_armed &&
        !events->snapshot.marker_wide &&
        ((uint32_t) (input->now_ms - events->left_start_ms) >=
            events->config.finish_rearm_ms)) {
        events->snapshot.finish_armed = true;
    }

    if (events->snapshot.finish_armed &&
        !events->snapshot.finish_a_passed &&
        events->marker_rise_event) {
        events->snapshot.finish_a_passed = true;
        events->snapshot.finish_a_passed_event = true;
    }
}

bool HRouteEvents_GetSnapshot(const h_route_events_t *events,
    h_route_snapshot_t *snapshot)
{
    if ((events == NULL) || (snapshot == NULL)) {
        return false;
    }
    *snapshot = events->snapshot;
    return true;
}

static void h_route_events_reset_run(h_route_events_t *events)
{
    events->snapshot.input_valid = false;
    events->snapshot.running = false;
    events->snapshot.marker_wide = false;
    events->snapshot.initial_a_seen = false;
    events->snapshot.left_start_a = false;
    events->snapshot.finish_armed = false;
    events->snapshot.finish_a_passed = false;
    events->snapshot.left_start_a_event = false;
    events->snapshot.finish_a_passed_event = false;
    events->snapshot.start_count = 0;
    events->snapshot.progress_count = 0;
    events->snapshot.marker_count = 0U;
    events->raw_marker_wide = false;
    events->marker_rise_event = false;
    events->raw_marker_changed_ms = 0U;
    events->left_start_ms = 0U;
}

static void h_route_events_update_marker(h_route_events_t *events,
    const h_route_input_t *input)
{
    bool raw_wide = input->line_active_count >=
        events->config.marker_active_min;
    uint32_t debounce_ms = raw_wide ?
        events->config.marker_confirm_ms :
        events->config.marker_release_ms;

    if (raw_wide != events->raw_marker_wide) {
        events->raw_marker_wide = raw_wide;
        events->raw_marker_changed_ms = input->now_ms;
    }
    if ((events->snapshot.marker_wide != raw_wide) &&
        ((uint32_t) (input->now_ms -
            events->raw_marker_changed_ms) >= debounce_ms)) {
        events->snapshot.marker_wide = raw_wide;
        if (raw_wide) {
            events->snapshot.marker_count++;
            events->marker_rise_event = true;
        }
    }
}
