#ifndef APP_LINE_TRACKING_APP_H
#define APP_LINE_TRACKING_APP_H

#include "line_tracking_control.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call once after Motor_Init(), Encoder_Init(), Key_Init(), and before
 * the main loop. */
void LineTrackingApp_Init(uint32_t now_ms);

/* Call periodically after Encoder_Task() and Key_Task(). Handles the B21
 * start/stop toggle, DOWN stop, LCD line status, and optional telemetry. */
void LineTrackingApp_Task(uint32_t now_ms);

void LineTrackingApp_Start(void);
void LineTrackingApp_Stop(void);
void LineTrackingApp_Toggle(void);
void LineTrackingApp_SetEnabled(bool enabled);
bool LineTrackingApp_IsEnabled(void);

/* Disabled by default. Enable only while tuning to print:
 * d:leftTarget,rightTarget,leftActual,rightActual,lineErr,turnPps */
void LineTrackingApp_SetTelemetryEnabled(bool enabled);
void LineTrackingApp_GetState(line_tracking_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
