#include "h_route_events.h"

#include <assert.h>
#include <string.h>

static h_route_config_t valid_config(void)
{
    const h_route_config_t config = {
        .precision_stop_delta_count = 0,
        .finish_rearm_ms = 300U,
        .marker_active_min = 5U,
        .marker_confirm_ms = 20U,
        .marker_release_ms = 20U
    };
    return config;
}

static h_route_input_t route_input(uint32_t now_ms,
    int32_t count, uint8_t active_count)
{
    h_route_input_t input;

    memset(&input, 0, sizeof(input));
    input.now_ms = now_ms;
    input.odometry_count = count;
    input.line_active_count = active_count;
    input.odometry_ready = true;
    input.line_sensor_ready = true;
    return input;
}

static void update(h_route_events_t *events, uint32_t now_ms,
    int32_t count, uint8_t active_count)
{
    h_route_input_t input = route_input(now_ms, count, active_count);
    HRouteEvents_Update(events, &input);
}

static void confirm_initial_a_and_leave(h_route_events_t *events)
{
    update(events, 0U, 0, 5U);
    update(events, 20U, 0, 5U);
    assert(events->snapshot.initial_a_seen);
    assert(events->snapshot.marker_wide);

    update(events, 30U, 10, 2U);
    update(events, 50U, 25, 2U);
    assert(events->snapshot.left_start_a_event);
    assert(events->snapshot.left_start_a);
}

static void test_invalid_config_blocks_start(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    config.finish_rearm_ms = 0U;
    HRouteEvents_Init(&events, &config);
    assert(!events.snapshot.configuration_valid);
    assert(!HRouteEvents_Start(&events, 0U, 0));

    config = valid_config();
    config.precision_stop_delta_count = 2001;
    assert(!HRouteEvents_SetConfig(&events, &config));
}

static void test_start_marker_is_visible_before_run(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    update(&events, 0U, 0, 4U);
    update(&events, 50U, 0, 4U);
    assert(!events.snapshot.marker_wide);
    update(&events, 60U, 0, 5U);
    update(&events, 79U, 0, 5U);
    assert(!events.snapshot.marker_wide);
    update(&events, 80U, 0, 5U);
    assert(events.snapshot.marker_wide);
    assert(!events.snapshot.initial_a_seen);
    assert(!events.snapshot.running);
}

static void test_initial_a_needs_debounce_and_release(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 100));
    update(&events, 0U, 100, 5U);
    update(&events, 19U, 100, 5U);
    assert(!events.snapshot.initial_a_seen);
    update(&events, 20U, 100, 5U);
    assert(events.snapshot.initial_a_seen);

    update(&events, 30U, 130, 2U);
    update(&events, 49U, 130, 2U);
    assert(!events.snapshot.left_start_a);
    update(&events, 50U, 130, 2U);
    assert(events.snapshot.left_start_a_event);
    update(&events, 51U, 131, 2U);
    assert(!events.snapshot.left_start_a_event);
}

static void test_finish_requires_rearm_and_new_wide_edge(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    confirm_initial_a_and_leave(&events);

    update(&events, 100U, 200, 5U);
    update(&events, 120U, 200, 5U);
    assert(!events.snapshot.finish_armed);
    assert(!events.snapshot.finish_a_passed);

    update(&events, 130U, 220, 2U);
    update(&events, 150U, 220, 2U);
    update(&events, 349U, 500, 2U);
    assert(!events.snapshot.finish_armed);
    update(&events, 350U, 501, 2U);
    assert(events.snapshot.finish_armed);

    update(&events, 360U, 520, 5U);
    update(&events, 379U, 520, 5U);
    assert(!events.snapshot.finish_a_passed);
    update(&events, 380U, 520, 5U);
    assert(events.snapshot.finish_a_passed_event);
    assert(events.snapshot.finish_a_passed);
    assert(events.snapshot.marker_count == 3U);
    update(&events, 381U, 521, 5U);
    assert(!events.snapshot.finish_a_passed_event);
}

static void test_missing_initial_a_never_arms_finish(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    update(&events, 0U, 100, 2U);
    update(&events, 1000U, 500, 2U);
    assert(!events.snapshot.initial_a_seen);
    assert(!events.snapshot.left_start_a);
    assert(!events.snapshot.finish_armed);
}

static void test_invalid_input_suppresses_events(void)
{
    h_route_events_t events;
    h_route_config_t config = valid_config();
    h_route_input_t input = route_input(0U, 0, 5U);

    HRouteEvents_Init(&events, &config);
    assert(HRouteEvents_Start(&events, 0U, 0));
    input.line_sensor_ready = false;
    HRouteEvents_Update(&events, &input);
    input.now_ms = 100U;
    HRouteEvents_Update(&events, &input);
    assert(!events.snapshot.input_valid);
    assert(!events.snapshot.initial_a_seen);

    input.line_sensor_ready = true;
    HRouteEvents_Update(&events, &input);
    input.now_ms = 119U;
    HRouteEvents_Update(&events, &input);
    assert(!events.snapshot.initial_a_seen);
    input.now_ms = 120U;
    HRouteEvents_Update(&events, &input);
    assert(events.snapshot.initial_a_seen);
}

int main(void)
{
    test_invalid_config_blocks_start();
    test_start_marker_is_visible_before_run();
    test_initial_a_needs_debounce_and_release();
    test_finish_requires_rearm_and_new_wide_edge();
    test_missing_initial_a_never_arms_finish();
    test_invalid_input_suppresses_events();
    return 0;
}
