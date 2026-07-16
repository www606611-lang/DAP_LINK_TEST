#include "encoder_input.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define ENCODER_0_A_IIDX ENCODER_GPIOB_ENCODER_0_A_IIDX
#define ENCODER_0_A_PIN  ENCODER_GPIOB_ENCODER_0_A_PIN
#define ENCODER_0_B_IIDX ENCODER_GPIOB_ENCODER_0_B_IIDX
#define ENCODER_0_B_PIN  ENCODER_GPIOB_ENCODER_0_B_PIN
#define ENCODER_1_A_IIDX ENCODER_GPIOB_ENCODER_1_A_IIDX
#define ENCODER_1_A_PIN  ENCODER_GPIOB_ENCODER_1_A_PIN
#define ENCODER_1_B_IIDX ENCODER_GPIOB_ENCODER_1_B_IIDX
#define ENCODER_1_B_PIN  ENCODER_GPIOB_ENCODER_1_B_PIN

#define ENCODER_INPUT_SPEED_FILTER_SAMPLES 4U

typedef struct {
    volatile int32_t count;
    volatile uint32_t edge_count;
    volatile uint32_t invalid_transition_count;
    volatile uint8_t last_state;
    int32_t last_sample_count;
    int32_t delta;
    int32_t speed_pps;
    int32_t speed_history[ENCODER_INPUT_SPEED_FILTER_SAMPLES];
    int64_t speed_sum;
    uint8_t speed_history_index;
    uint8_t speed_history_count;
    int8_t direction;
    bool inverted;
} encoder_input_state_t;

static encoder_input_state_t g_encoder[ENCODER_INPUT_COUNT];
static uint32_t g_last_sample_ms;

static bool encoder_input_is_valid_id(encoder_input_id_t id);
static void encoder_input_record_edge(encoder_input_id_t id);
static uint8_t encoder_input_read_state(encoder_input_id_t id);
static uint32_t encoder_input_enter_critical(void);
static void encoder_input_exit_critical(uint32_t primask);
static int8_t encoder_input_direction_from_delta(int32_t delta);
static int32_t encoder_input_filter_speed(
    encoder_input_state_t *encoder, int32_t raw_speed_pps);

void EncoderInput_Init(uint32_t now_ms)
{
    uint32_t primask = encoder_input_enter_critical();

    g_encoder[ENCODER_INPUT_0].last_state =
        encoder_input_read_state(ENCODER_INPUT_0);
    g_encoder[ENCODER_INPUT_1].last_state =
        encoder_input_read_state(ENCODER_INPUT_1);
    encoder_input_exit_critical(primask);

    g_last_sample_ms = now_ms;
}

void EncoderInput_Task(uint32_t now_ms)
{
    int32_t counts[ENCODER_INPUT_COUNT];
    uint32_t elapsed_ms = now_ms - g_last_sample_ms;
    uint32_t primask;
    uint32_t i;

    if (elapsed_ms < ENCODER_INPUT_SAMPLE_INTERVAL_MS) {
        return;
    }

    primask = encoder_input_enter_critical();
    for (i = 0U; i < (uint32_t) ENCODER_INPUT_COUNT; i++) {
        counts[i] = g_encoder[i].count;
    }
    encoder_input_exit_critical(primask);

    for (i = 0U; i < (uint32_t) ENCODER_INPUT_COUNT; i++) {
        encoder_input_state_t *encoder = &g_encoder[i];
        int32_t delta = counts[i] - encoder->last_sample_count;
        int32_t raw_speed_pps;

        encoder->last_sample_count = counts[i];
        encoder->delta = delta;
        raw_speed_pps =
            (int32_t) (((int64_t) delta * 1000) / elapsed_ms);
        encoder->speed_pps = encoder_input_filter_speed(
            encoder, raw_speed_pps);
        encoder->direction = encoder_input_direction_from_delta(delta);
    }

    g_last_sample_ms = now_ms;
}

void EncoderInput_SetInverted(encoder_input_id_t id, bool inverted)
{
    uint32_t primask;

    if (!encoder_input_is_valid_id(id)) {
        return;
    }

    primask = encoder_input_enter_critical();
    g_encoder[id].inverted = inverted;
    encoder_input_exit_critical(primask);
}

void EncoderInput_Reset(encoder_input_id_t id)
{
    uint32_t primask;
    uint32_t i;

    if (!encoder_input_is_valid_id(id)) {
        return;
    }

    primask = encoder_input_enter_critical();
    g_encoder[id].count = 0;
    g_encoder[id].edge_count = 0U;
    g_encoder[id].invalid_transition_count = 0U;
    g_encoder[id].last_state = encoder_input_read_state(id);
    encoder_input_exit_critical(primask);

    g_encoder[id].last_sample_count = 0;
    g_encoder[id].delta = 0;
    g_encoder[id].speed_pps = 0;
    for (i = 0U; i < ENCODER_INPUT_SPEED_FILTER_SAMPLES; i++) {
        g_encoder[id].speed_history[i] = 0;
    }
    g_encoder[id].speed_sum = 0;
    g_encoder[id].speed_history_index = 0U;
    g_encoder[id].speed_history_count = 0U;
    g_encoder[id].direction = 0;
}

void EncoderInput_ResetAll(void)
{
    EncoderInput_Reset(ENCODER_INPUT_0);
    EncoderInput_Reset(ENCODER_INPUT_1);
}

bool EncoderInput_GetSnapshot(
    encoder_input_id_t id, encoder_input_snapshot_t *snapshot)
{
    uint32_t primask;

    if ((!encoder_input_is_valid_id(id)) || (snapshot == NULL)) {
        return false;
    }

    primask = encoder_input_enter_critical();
    snapshot->count = g_encoder[id].count;
    snapshot->edge_count = g_encoder[id].edge_count;
    snapshot->invalid_transition_count =
        g_encoder[id].invalid_transition_count;
    encoder_input_exit_critical(primask);

    snapshot->delta = g_encoder[id].delta;
    snapshot->speed_pps = g_encoder[id].speed_pps;
    snapshot->direction = g_encoder[id].direction;
    return true;
}

bool EncoderInput_OnGpioInterrupt(uint32_t interrupt_index)
{
    switch (interrupt_index) {
        case ENCODER_0_A_IIDX:
        case ENCODER_0_B_IIDX:
            encoder_input_record_edge(ENCODER_INPUT_0);
            return true;
        case ENCODER_1_A_IIDX:
        case ENCODER_1_B_IIDX:
            encoder_input_record_edge(ENCODER_INPUT_1);
            return true;
        default:
            return false;
    }
}

static bool encoder_input_is_valid_id(encoder_input_id_t id)
{
    return ((uint32_t) id < (uint32_t) ENCODER_INPUT_COUNT);
}

static void encoder_input_record_edge(encoder_input_id_t id)
{
    static const int8_t step_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };
    uint8_t previous_state = g_encoder[id].last_state;
    uint8_t current_state = encoder_input_read_state(id);
    uint8_t transition =
        (uint8_t) ((previous_state << 2U) | current_state);
    int8_t step = step_table[transition];

    g_encoder[id].edge_count++;
    g_encoder[id].last_state = current_state;

    if ((previous_state ^ current_state) == 0x03U) {
        g_encoder[id].invalid_transition_count++;
        return;
    }

    if (g_encoder[id].inverted) {
        step = (int8_t) -step;
    }
    g_encoder[id].count += step;
}

static uint8_t encoder_input_read_state(encoder_input_id_t id)
{
    uint32_t a_pin;
    uint32_t b_pin;
    uint32_t pins;

    if (id == ENCODER_INPUT_0) {
        a_pin = ENCODER_0_A_PIN;
        b_pin = ENCODER_0_B_PIN;
    } else {
        a_pin = ENCODER_1_A_PIN;
        b_pin = ENCODER_1_B_PIN;
    }

    pins = DL_GPIO_readPins(ENCODER_GPIOB_PORT, a_pin | b_pin);
    return (uint8_t) (((pins & a_pin) != 0U) ? 2U : 0U) |
        (uint8_t) (((pins & b_pin) != 0U) ? 1U : 0U);
}

static uint32_t encoder_input_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void encoder_input_exit_critical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static int8_t encoder_input_direction_from_delta(int32_t delta)
{
    if (delta > 0) {
        return 1;
    }
    if (delta < 0) {
        return -1;
    }
    return 0;
}

static int32_t encoder_input_filter_speed(
    encoder_input_state_t *encoder, int32_t raw_speed_pps)
{
    uint8_t index = encoder->speed_history_index;

    if (encoder->speed_history_count <
        ENCODER_INPUT_SPEED_FILTER_SAMPLES) {
        encoder->speed_history_count++;
    } else {
        encoder->speed_sum -= encoder->speed_history[index];
    }

    encoder->speed_history[index] = raw_speed_pps;
    encoder->speed_sum += raw_speed_pps;
    index++;
    if (index >= ENCODER_INPUT_SPEED_FILTER_SAMPLES) {
        index = 0U;
    }
    encoder->speed_history_index = index;

    return (int32_t) (encoder->speed_sum /
        encoder->speed_history_count);
}
