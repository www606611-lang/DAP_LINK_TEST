#ifndef APP_RUNTIME_METRICS_H
#define APP_RUNTIME_METRICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t loop_interval_ms;
    uint32_t loop_max_interval_ms;
    uint32_t display_duration_ms;
    uint32_t display_max_duration_ms;
} app_runtime_metrics_snapshot_t;

void AppRuntimeMetrics_Init(uint32_t now_ms);
void AppRuntimeMetrics_Reset(uint32_t now_ms);
void AppRuntimeMetrics_RecordLoop(uint32_t now_ms);
void AppRuntimeMetrics_RecordDisplay(uint32_t duration_ms);
bool AppRuntimeMetrics_GetSnapshot(
    app_runtime_metrics_snapshot_t *snapshot);

#endif
