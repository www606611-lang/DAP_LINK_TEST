#ifndef DRIVERS_MCU_ENCODER_INPUT_H
#define DRIVERS_MCU_ENCODER_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_INPUT_SAMPLE_INTERVAL_MS 50U

typedef enum {
    ENCODER_INPUT_0 = 0,
    ENCODER_INPUT_1,
    ENCODER_INPUT_COUNT
} encoder_input_id_t;

typedef struct {
    int32_t count;
    int32_t delta;
    int32_t speed_pps;
    int8_t direction;
    uint32_t edge_count;
    uint32_t invalid_transition_count;
} encoder_input_snapshot_t;

void EncoderInput_Init(uint32_t now_ms);
void EncoderInput_Task(uint32_t now_ms);
void EncoderInput_SetInverted(encoder_input_id_t id, bool inverted);
void EncoderInput_Reset(encoder_input_id_t id);
void EncoderInput_ResetAll(void);
bool EncoderInput_GetSnapshot(
    encoder_input_id_t id, encoder_input_snapshot_t *snapshot);

#endif
