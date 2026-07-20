#ifndef DRIVERS_DEVICE_K230_VISION_LINK_H
#define DRIVERS_DEVICE_K230_VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define K230_VISION_IMAGE_WIDTH 400U
#define K230_VISION_IMAGE_HEIGHT 240U
#define K230_VISION_CENTER_X 200
#define K230_VISION_CENTER_Y 120
#define K230_VISION_OFFLINE_TIMEOUT_MS 150U

typedef struct {
    bool online;
    bool target_valid;
    uint16_t cx;
    uint16_t cy;
    int16_t error_x;
    int16_t error_y;
    uint32_t frame_sequence;
    uint32_t last_frame_ms;
    uint32_t parse_error_count;
    uint32_t resync_count;
    uint32_t timeout_count;
    uint32_t received_byte_count;
    uint32_t overflow_count;
} k230_vision_snapshot_t;

void K230VisionLink_Init(uint32_t now_ms);
void K230VisionLink_Task(uint32_t now_ms);
bool K230VisionLink_GetSnapshot(k230_vision_snapshot_t *snapshot);

#endif
