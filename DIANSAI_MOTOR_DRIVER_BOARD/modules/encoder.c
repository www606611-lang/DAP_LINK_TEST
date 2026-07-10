#include "encoder.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define ENCODER_PORT Encoder_Input_Pin_PORT
#define ENCODER_LEFT_A_IIDX Encoder_Input_Pin_Encoder_L_A_IIDX
#define ENCODER_LEFT_A_PIN  Encoder_Input_Pin_Encoder_L_A_PIN
#define ENCODER_LEFT_B_IIDX Encoder_Input_Pin_Encoder_L_B_IIDX
#define ENCODER_LEFT_B_PIN  Encoder_Input_Pin_Encoder_L_B_PIN
#define ENCODER_RIGHT_A_IIDX Encoder_Input_Pin_Encoder_R_A_IIDX
#define ENCODER_RIGHT_A_PIN  Encoder_Input_Pin_Encoder_R_A_PIN
#define ENCODER_RIGHT_B_IIDX Encoder_Input_Pin_Encoder_R_B_IIDX
#define ENCODER_RIGHT_B_PIN  Encoder_Input_Pin_Encoder_R_B_PIN

#ifdef Encoder_Input_Pin_INT_IRQN
#define ENCODER_INT_IRQN Encoder_Input_Pin_INT_IRQN
#define ENCODER_INT_IIDX Encoder_Input_Pin_INT_IIDX
#else
#define ENCODER_INT_IRQN GPIO_MULTIPLE_GPIOB_INT_IRQN
#define ENCODER_INT_IIDX GPIO_MULTIPLE_GPIOB_INT_IIDX
#endif

typedef struct {
    volatile int32_t count;
    uint8_t last_state;
    int32_t last_count;
    int32_t delta;
    int32_t speed_pps;
    int8_t direction;
    bool inverted;
} encoder_state_t;

static encoder_state_t g_encoder[ENCODER_ID_COUNT];
static uint32_t g_last_sample_ms;

static bool encoder_is_valid_id(encoder_id_t id);
static void encoder_record_edge(encoder_id_t id);
static uint8_t encoder_read_state(encoder_id_t id);
static int32_t encoder_read_count(encoder_id_t id);
static uint32_t encoder_enter_critical(void);
static void encoder_exit_critical(uint32_t primask);
static int8_t encoder_direction_from_delta(int32_t delta);

void Encoder_Init(uint32_t now_ms)
{
    Encoder_ResetAll();
    g_encoder[ENCODER_LEFT].last_state = encoder_read_state(ENCODER_LEFT);
    g_encoder[ENCODER_RIGHT].last_state = encoder_read_state(ENCODER_RIGHT);
    g_last_sample_ms = now_ms;
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

void Encoder_Task(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - g_last_sample_ms;

    if (elapsed_ms < ENCODER_SAMPLE_INTERVAL_MS) {
        return;
    }

    for (uint32_t i = 0U; i < (uint32_t) ENCODER_ID_COUNT; i++) {
        encoder_state_t *encoder = &g_encoder[i];
        int32_t count = encoder_read_count((encoder_id_t) i);
        int32_t delta = count - encoder->last_count;

        encoder->last_count = count;
        encoder->delta = delta;
        encoder->speed_pps = (int32_t) (((int64_t) delta * 1000) / elapsed_ms);
        encoder->direction = encoder_direction_from_delta(delta);
    }

    g_last_sample_ms = now_ms;
}

void Encoder_Reset(encoder_id_t id)
{
    uint32_t primask;

    if (!encoder_is_valid_id(id)) {
        return;
    }

    primask = encoder_enter_critical();
    g_encoder[id].count = 0;
    g_encoder[id].last_state = encoder_read_state(id);
    encoder_exit_critical(primask);

    g_encoder[id].last_count = 0;
    g_encoder[id].delta = 0;
    g_encoder[id].speed_pps = 0;
    g_encoder[id].direction = 0;
}

void Encoder_ResetAll(void)
{
    Encoder_Reset(ENCODER_LEFT);
    Encoder_Reset(ENCODER_RIGHT);
}

void Encoder_SetInverted(encoder_id_t id, bool inverted)
{
    if (!encoder_is_valid_id(id)) {
        return;
    }

    g_encoder[id].inverted = inverted;
}

int32_t Encoder_GetCount(encoder_id_t id)
{
    if (!encoder_is_valid_id(id)) {
        return 0;
    }

    return encoder_read_count(id);
}

int32_t Encoder_GetDelta(encoder_id_t id)
{
    if (!encoder_is_valid_id(id)) {
        return 0;
    }

    return g_encoder[id].delta;
}

int32_t Encoder_GetSpeedPps(encoder_id_t id)
{
    if (!encoder_is_valid_id(id)) {
        return 0;
    }

    return g_encoder[id].speed_pps;
}

int8_t Encoder_GetDirection(encoder_id_t id)
{
    if (!encoder_is_valid_id(id)) {
        return 0;
    }

    return g_encoder[id].direction;
}

bool Encoder_GetSnapshot(encoder_id_t id, encoder_snapshot_t *snapshot)
{
    if ((!encoder_is_valid_id(id)) || (snapshot == NULL)) {
        return false;
    }

    snapshot->count = encoder_read_count(id);
    snapshot->delta = g_encoder[id].delta;
    snapshot->speed_pps = g_encoder[id].speed_pps;
    snapshot->direction = g_encoder[id].direction;
    return true;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case ENCODER_INT_IIDX:
            while (true) {
                switch (DL_GPIO_getPendingInterrupt(ENCODER_PORT)) {
                    case ENCODER_LEFT_A_IIDX:
                    case ENCODER_LEFT_B_IIDX:
                        encoder_record_edge(ENCODER_LEFT);
                        break;
                    case ENCODER_RIGHT_A_IIDX:
                    case ENCODER_RIGHT_B_IIDX:
                        encoder_record_edge(ENCODER_RIGHT);
                        break;
                    case DL_GPIO_IIDX_NO_INTR:
                        return;
                    default:
                        return;
                }
            }
        default:
            break;
    }
}

static bool encoder_is_valid_id(encoder_id_t id)
{
    return ((uint32_t) id < (uint32_t) ENCODER_ID_COUNT);
}

static void encoder_record_edge(encoder_id_t id)
{
    static const int8_t step_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };
    uint8_t state = encoder_read_state(id);
    uint8_t index = (uint8_t) ((g_encoder[id].last_state << 2U) | state);
    int32_t step = step_table[index];

    g_encoder[id].last_state = state;

    if (g_encoder[id].inverted) {
        step = -step;
    }

    g_encoder[id].count += step;
}

static uint8_t encoder_read_state(encoder_id_t id)
{
    uint32_t pins;
    uint32_t a_pin;
    uint32_t b_pin;

    if (id == ENCODER_LEFT) {
        a_pin = ENCODER_LEFT_A_PIN;
        b_pin = ENCODER_LEFT_B_PIN;
    } else {
        a_pin = ENCODER_RIGHT_A_PIN;
        b_pin = ENCODER_RIGHT_B_PIN;
    }

    pins = DL_GPIO_readPins(ENCODER_PORT, a_pin | b_pin);

    return (uint8_t) (((pins & a_pin) != 0U) ? 2U : 0U) |
           (uint8_t) (((pins & b_pin) != 0U) ? 1U : 0U);
}

static int32_t encoder_read_count(encoder_id_t id)
{
    int32_t count;
    uint32_t primask = encoder_enter_critical();

    count = g_encoder[id].count;
    encoder_exit_critical(primask);

    return count;
}

static uint32_t encoder_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void encoder_exit_critical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static int8_t encoder_direction_from_delta(int32_t delta)
{
    if (delta > 0) {
        return 1;
    }
    if (delta < 0) {
        return -1;
    }
    return 0;
}
