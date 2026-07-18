#ifndef APP_DISPLAY_CAR_DISPLAY_H
#define APP_DISPLAY_CAR_DISPLAY_H

#include <stdint.h>

#define CAR_DISPLAY_SLICE_INTERVAL_MS 20U

typedef enum {
    CAR_DISPLAY_PHASE_HEADER = 0,
    CAR_DISPLAY_PHASE_SPEED,
    CAR_DISPLAY_PHASE_ENCODER,
    CAR_DISPLAY_PHASE_ATTITUDE,
    CAR_DISPLAY_PHASE_LINE,
    CAR_DISPLAY_PHASE_CONTROL,
    CAR_DISPLAY_PHASE_HEALTH,
    CAR_DISPLAY_PHASE_FOOTER,
    CAR_DISPLAY_PHASE_COUNT
} car_display_phase_t;

void CarDisplay_Init(void);
void CarDisplay_Update(uint32_t now_ms, car_display_phase_t phase);

#endif
