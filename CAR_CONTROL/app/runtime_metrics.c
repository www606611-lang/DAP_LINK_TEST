#include "runtime_metrics.h"

#include <stddef.h>

static app_runtime_metrics_snapshot_t g_metrics;
static uint32_t g_last_loop_ms;

void AppRuntimeMetrics_Init(uint32_t now_ms)
{
    AppRuntimeMetrics_Reset(now_ms);
}

void AppRuntimeMetrics_Reset(uint32_t now_ms)
{
    g_metrics.loop_interval_ms = 0U;
    g_metrics.loop_max_interval_ms = 0U;
    g_metrics.display_duration_ms = 0U;
    g_metrics.display_max_duration_ms = 0U;
    g_last_loop_ms = now_ms;
}

void AppRuntimeMetrics_RecordLoop(uint32_t now_ms)
{
    uint32_t interval_ms = now_ms - g_last_loop_ms;

    g_last_loop_ms = now_ms;
    g_metrics.loop_interval_ms = interval_ms;
    if (interval_ms > g_metrics.loop_max_interval_ms) {
        g_metrics.loop_max_interval_ms = interval_ms;
    }
}

void AppRuntimeMetrics_RecordDisplay(uint32_t duration_ms)
{
    g_metrics.display_duration_ms = duration_ms;
    if (duration_ms > g_metrics.display_max_duration_ms) {
        g_metrics.display_max_duration_ms = duration_ms;
    }
}

bool AppRuntimeMetrics_GetSnapshot(app_runtime_metrics_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_metrics;
    return true;
}
