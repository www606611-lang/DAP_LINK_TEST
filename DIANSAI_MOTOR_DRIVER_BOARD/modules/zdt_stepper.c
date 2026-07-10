#include "zdt_stepper.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define ZDT_CHECK_BYTE          (0x6BU)
#define ZDT_TX_BUFFER_INDEX     (0U)
#define ZDT_TX_BUFFER_MASK      (1UL << ZDT_TX_BUFFER_INDEX)
#define ZDT_CAN_TIMEOUT_LOOPS   (1000000UL)
#define ZDT_CAN_EXT_ID_MASK     (0x1FFFFFFFUL)

#define ZDT_FUNC_RESET_POS      (0x0AU)
#define ZDT_FUNC_CLEAR_STALL    (0x0EU)
#define ZDT_FUNC_ENABLE         (0xF3U)
#define ZDT_FUNC_VELOCITY       (0xF6U)
#define ZDT_FUNC_POSITION       (0xFDU)
#define ZDT_FUNC_STOP           (0xFEU)
#define ZDT_FUNC_SYNC           (0xFFU)
#define ZDT_FUNC_SET_ORIGIN     (0x93U)
#define ZDT_FUNC_TRIGGER_ORIGIN (0x9AU)
#define ZDT_FUNC_ABORT_ORIGIN   (0x9CU)

typedef struct {
    uint8_t address;
    bool inverted;
    int16_t speed_rpm;
    int32_t pulse_count;
} zdt_stepper_state_t;

static zdt_stepper_state_t g_stepper[ZDT_STEPPER_COUNT] = {
    {1U, false, 0, 0},
    {2U, false, 0, 0},
};

static bool g_can_ready;

static bool zdt_is_valid_motor(zdt_stepper_id_t motor);
static bool zdt_wait_op_mode(uint32_t mode);
static bool zdt_wait_tx_ready(void);
static void zdt_configure_can_rx(void);
static bool zdt_send_frame(uint32_t ext_id, const uint8_t *data, uint8_t len);
static bool zdt_send_cmd(const uint8_t *cmd, uint8_t len);
static uint16_t zdt_limit_rpm(uint16_t rpm);
static uint32_t zdt_abs_i32(int32_t value);
static uint8_t zdt_dir_from_signed(zdt_stepper_id_t motor, int32_t value);
static bool zdt_pos_control(zdt_stepper_id_t motor, int32_t pulses,
    uint16_t rpm, uint8_t acc, bool absolute, bool sync);
static bool zdt_cmd_for_param(zdt_stepper_param_t param, uint8_t *cmd,
    uint8_t *len);
static uint8_t zdt_dlc_to_len(uint32_t dlc);

void ZdtStepper_Init(void)
{
    if (!zdt_wait_op_mode(DL_MCAN_OPERATION_MODE_NORMAL)) {
        g_can_ready = false;
        return;
    }

    zdt_configure_can_rx();
    g_can_ready = zdt_wait_op_mode(DL_MCAN_OPERATION_MODE_NORMAL);
}

bool ZdtStepper_SetAddress(zdt_stepper_id_t motor, uint8_t address)
{
    if ((!zdt_is_valid_motor(motor)) ||
        (address == ZDT_STEPPER_ADDR_BROADCAST)) {
        return false;
    }

    g_stepper[motor].address = address;
    return true;
}

uint8_t ZdtStepper_GetAddress(zdt_stepper_id_t motor)
{
    if (!zdt_is_valid_motor(motor)) {
        return 0U;
    }

    return g_stepper[motor].address;
}

void ZdtStepper_SetInverted(zdt_stepper_id_t motor, bool inverted)
{
    if (zdt_is_valid_motor(motor)) {
        g_stepper[motor].inverted = inverted;
    }
}

bool ZdtStepper_IsInverted(zdt_stepper_id_t motor)
{
    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    return g_stepper[motor].inverted;
}

bool ZdtStepper_Enable(zdt_stepper_id_t motor, bool enable)
{
    uint8_t cmd[6];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_ENABLE;
    cmd[2] = 0xABU;
    cmd[3] = enable ? 1U : 0U;
    cmd[4] = 0U;
    cmd[5] = ZDT_CHECK_BYTE;

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_EnableAll(bool enable)
{
    bool ok = true;

    ok = ZdtStepper_Enable(ZDT_STEPPER_1, enable) && ok;
    ok = ZdtStepper_Enable(ZDT_STEPPER_2, enable) && ok;
    return ok;
}

bool ZdtStepper_SetSpeed(zdt_stepper_id_t motor, int16_t rpm, uint8_t acc)
{
    return ZdtStepper_SetSpeedSync(motor, rpm, acc, false);
}

bool ZdtStepper_SetSpeedSync(
    zdt_stepper_id_t motor, int16_t rpm, uint8_t acc, bool sync)
{
    uint16_t speed;
    uint8_t cmd[8];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    speed = zdt_limit_rpm((uint16_t) zdt_abs_i32((int32_t) rpm));

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_VELOCITY;
    cmd[2] = zdt_dir_from_signed(motor, (int32_t) rpm);
    cmd[3] = (uint8_t) (speed >> 8);
    cmd[4] = (uint8_t) speed;
    cmd[5] = acc;
    cmd[6] = sync ? 1U : 0U;
    cmd[7] = ZDT_CHECK_BYTE;

    if (zdt_send_cmd(cmd, sizeof(cmd))) {
        g_stepper[motor].speed_rpm = rpm;
        g_stepper[motor].pulse_count = 0;
        return true;
    }

    return false;
}

int16_t ZdtStepper_GetTargetSpeedRpm(zdt_stepper_id_t motor)
{
    if (!zdt_is_valid_motor(motor)) {
        return 0;
    }

    return g_stepper[motor].speed_rpm;
}

int32_t ZdtStepper_GetTargetPulseCount(zdt_stepper_id_t motor)
{
    if (!zdt_is_valid_motor(motor)) {
        return 0;
    }

    return g_stepper[motor].pulse_count;
}

bool ZdtStepper_MoveRelative(
    zdt_stepper_id_t motor, int32_t pulses, uint16_t rpm, uint8_t acc,
    bool sync)
{
    return zdt_pos_control(motor, pulses, rpm, acc, false, sync);
}

bool ZdtStepper_MoveAbsolute(
    zdt_stepper_id_t motor, int32_t position, uint16_t rpm, uint8_t acc,
    bool sync)
{
    return zdt_pos_control(motor, position, rpm, acc, true, sync);
}

bool ZdtStepper_StartSync(void)
{
    const uint8_t cmd[4] = {
        ZDT_STEPPER_ADDR_BROADCAST,
        ZDT_FUNC_SYNC,
        0x66U,
        ZDT_CHECK_BYTE,
    };

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_Stop(zdt_stepper_id_t motor, bool sync)
{
    uint8_t cmd[5];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_STOP;
    cmd[2] = 0x98U;
    cmd[3] = sync ? 1U : 0U;
    cmd[4] = ZDT_CHECK_BYTE;

    if (zdt_send_cmd(cmd, sizeof(cmd))) {
        g_stepper[motor].speed_rpm = 0;
        g_stepper[motor].pulse_count = 0;
        return true;
    }

    return false;
}

bool ZdtStepper_StopAll(void)
{
    bool ok = true;

    ok = ZdtStepper_Stop(ZDT_STEPPER_1, false) && ok;
    ok = ZdtStepper_Stop(ZDT_STEPPER_2, false) && ok;
    return ok;
}

bool ZdtStepper_ResetPosition(zdt_stepper_id_t motor)
{
    uint8_t cmd[4];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_RESET_POS;
    cmd[2] = 0x6DU;
    cmd[3] = ZDT_CHECK_BYTE;

    if (zdt_send_cmd(cmd, sizeof(cmd))) {
        g_stepper[motor].pulse_count = 0;
        return true;
    }

    return false;
}

bool ZdtStepper_ClearStall(zdt_stepper_id_t motor)
{
    uint8_t cmd[4];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_CLEAR_STALL;
    cmd[2] = 0x52U;
    cmd[3] = ZDT_CHECK_BYTE;

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_RequestParam(
    zdt_stepper_id_t motor, zdt_stepper_param_t param)
{
    uint8_t cmd[5];
    uint8_t len;

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    if (!zdt_cmd_for_param(param, cmd, &len)) {
        return false;
    }

    return zdt_send_cmd(cmd, len);
}

bool ZdtStepper_SetOriginHere(zdt_stepper_id_t motor, bool save)
{
    uint8_t cmd[5];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_SET_ORIGIN;
    cmd[2] = 0x88U;
    cmd[3] = save ? 1U : 0U;
    cmd[4] = ZDT_CHECK_BYTE;

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_TriggerOrigin(
    zdt_stepper_id_t motor, uint8_t mode, bool sync)
{
    uint8_t cmd[5];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_TRIGGER_ORIGIN;
    cmd[2] = mode;
    cmd[3] = sync ? 1U : 0U;
    cmd[4] = ZDT_CHECK_BYTE;

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_AbortOrigin(zdt_stepper_id_t motor)
{
    uint8_t cmd[4];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_ABORT_ORIGIN;
    cmd[2] = 0x48U;
    cmd[3] = ZDT_CHECK_BYTE;

    return zdt_send_cmd(cmd, sizeof(cmd));
}

bool ZdtStepper_ReadFrame(zdt_stepper_can_frame_t *frame)
{
    DL_MCAN_RxBufElement rx;
    DL_MCAN_RxFIFOStatus fifo;
    uint32_t ext_id;
    uint8_t len;

    if (frame == NULL) {
        return false;
    }

    fifo.num = DL_MCAN_RX_FIFO_NUM_0;
    fifo.fillLvl = 0U;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifo);

    if (fifo.fillLvl == 0U) {
        return false;
    }

    DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U, fifo.num, &rx);
    DL_MCAN_writeRxFIFOAck(MCAN0_INST, fifo.num, fifo.getIdx);

    if (rx.xtd != 0U) {
        ext_id = rx.id & ZDT_CAN_EXT_ID_MASK;
    } else {
        ext_id = (rx.id >> 18U) & 0x7FFU;
    }

    len = zdt_dlc_to_len(rx.dlc);
    memset(frame, 0, sizeof(*frame));
    frame->ext_id = ext_id;
    frame->address = (uint8_t) (ext_id >> 8);
    frame->packet = (uint8_t) ext_id;
    frame->dlc = len;
    memcpy(frame->data, rx.data, len);

    return true;
}

static bool zdt_is_valid_motor(zdt_stepper_id_t motor)
{
    return ((uint32_t) motor < (uint32_t) ZDT_STEPPER_COUNT);
}

static bool zdt_wait_op_mode(uint32_t mode)
{
    uint32_t timeout = ZDT_CAN_TIMEOUT_LOOPS;

    while (timeout > 0U) {
        if (DL_MCAN_getOpMode(MCAN0_INST) == mode) {
            return true;
        }
        timeout--;
    }

    return false;
}

static bool zdt_wait_tx_ready(void)
{
    uint32_t timeout = ZDT_CAN_TIMEOUT_LOOPS;

    while (timeout > 0U) {
        if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & ZDT_TX_BUFFER_MASK) == 0U) {
            return true;
        }
        timeout--;
    }

    return false;
}

static void zdt_configure_can_rx(void)
{
    DL_MCAN_ConfigParams config;

    memset(&config, 0, sizeof(config));
    config.timeoutSelect = DL_MCAN_TIMEOUT_SELECT_CONT;
    config.timeoutPreload = 0xFFFFU;
    config.filterConfig.rrfe = 1U;
    config.filterConfig.rrfs = 1U;
    config.filterConfig.anfe = 0U;
    config.filterConfig.anfs = 0U;

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_SW_INIT);
    if (!zdt_wait_op_mode(DL_MCAN_OPERATION_MODE_SW_INIT)) {
        return;
    }

    DL_MCAN_config(MCAN0_INST, &config);
    DL_MCAN_setExtIDAndMask(MCAN0_INST, ZDT_CAN_EXT_ID_MASK);
    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
}

static bool zdt_send_frame(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    DL_MCAN_TxBufElement tx;

    if ((data == NULL) || (len == 0U) || (len > 8U)) {
        return false;
    }

    if ((!g_can_ready) && (!zdt_wait_op_mode(DL_MCAN_OPERATION_MODE_NORMAL))) {
        return false;
    }

    if (!zdt_wait_tx_ready()) {
        return false;
    }

    memset(&tx, 0, sizeof(tx));
    tx.id = ext_id & ZDT_CAN_EXT_ID_MASK;
    tx.rtr = 0U;
    tx.xtd = 1U;
    tx.esi = 0U;
    tx.dlc = len;
    tx.brs = 0U;
    tx.fdf = 0U;
    tx.efc = 0U;
    tx.mm = 0U;
    memcpy(tx.data, data, len);

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
        ZDT_TX_BUFFER_INDEX, &tx);

    return (DL_MCAN_TXBufAddReq(MCAN0_INST, ZDT_TX_BUFFER_INDEX) == 0);
}

static bool zdt_send_cmd(const uint8_t *cmd, uint8_t len)
{
    uint8_t index = 0U;
    uint8_t packet = 0U;
    uint8_t payload_len;

    if ((cmd == NULL) || (len <= 2U)) {
        return false;
    }

    payload_len = (uint8_t) (len - 2U);
    while (index < payload_len) {
        uint8_t data[8] = {0};
        uint8_t take = (uint8_t) (payload_len - index);
        uint8_t i;

        if (take > 7U) {
            take = 7U;
        }

        data[0] = cmd[1];
        for (i = 0U; i < take; i++) {
            data[i + 1U] = cmd[index + 2U + i];
        }

        if (!zdt_send_frame(((uint32_t) cmd[0] << 8U) | packet, data,
                (uint8_t) (take + 1U))) {
            return false;
        }

        index = (uint8_t) (index + take);
        packet++;
    }

    return true;
}

static uint16_t zdt_limit_rpm(uint16_t rpm)
{
    if (rpm > ZDT_STEPPER_MAX_RPM) {
        return ZDT_STEPPER_MAX_RPM;
    }

    return rpm;
}

static uint32_t zdt_abs_i32(int32_t value)
{
    if (value < 0) {
        return (uint32_t) (-(value + 1)) + 1U;
    }

    return (uint32_t) value;
}

static uint8_t zdt_dir_from_signed(zdt_stepper_id_t motor, int32_t value)
{
    uint8_t dir = (value < 0) ? ZDT_STEPPER_DIR_CCW : ZDT_STEPPER_DIR_CW;

    if (g_stepper[motor].inverted) {
        dir ^= 1U;
    }

    return dir;
}

static bool zdt_pos_control(zdt_stepper_id_t motor, int32_t pulses,
    uint16_t rpm, uint8_t acc, bool absolute, bool sync)
{
    uint32_t count;
    uint16_t speed;
    uint8_t cmd[13];

    if (!zdt_is_valid_motor(motor)) {
        return false;
    }

    count = zdt_abs_i32(pulses);
    speed = zdt_limit_rpm(rpm);

    cmd[0] = g_stepper[motor].address;
    cmd[1] = ZDT_FUNC_POSITION;
    cmd[2] = zdt_dir_from_signed(motor, pulses);
    cmd[3] = (uint8_t) (speed >> 8);
    cmd[4] = (uint8_t) speed;
    cmd[5] = acc;
    cmd[6] = (uint8_t) (count >> 24);
    cmd[7] = (uint8_t) (count >> 16);
    cmd[8] = (uint8_t) (count >> 8);
    cmd[9] = (uint8_t) count;
    cmd[10] = absolute ? 1U : 0U;
    cmd[11] = sync ? 1U : 0U;
    cmd[12] = ZDT_CHECK_BYTE;

    if (zdt_send_cmd(cmd, sizeof(cmd))) {
        g_stepper[motor].speed_rpm = (pulses < 0) ? -(int16_t) speed :
                                                    (int16_t) speed;
        g_stepper[motor].pulse_count = pulses;
        return true;
    }

    return false;
}

static bool zdt_cmd_for_param(zdt_stepper_param_t param, uint8_t *cmd,
    uint8_t *len)
{
    if ((cmd == NULL) || (len == NULL)) {
        return false;
    }

    switch (param) {
        case ZDT_STEPPER_PARAM_VERSION:
            cmd[1] = 0x1FU;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_RL:
            cmd[1] = 0x20U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_PID:
            cmd[1] = 0x21U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_VBUS:
            cmd[1] = 0x24U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_PHASE_CURRENT:
            cmd[1] = 0x27U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_ENCODER:
            cmd[1] = 0x31U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_TARGET_POSITION:
            cmd[1] = 0x33U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_SPEED:
            cmd[1] = 0x35U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_POSITION:
            cmd[1] = 0x36U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_POSITION_ERROR:
            cmd[1] = 0x37U;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_FLAGS:
            cmd[1] = 0x3AU;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_ORIGIN:
            cmd[1] = 0x3BU;
            *len = 3U;
            break;
        case ZDT_STEPPER_PARAM_CONFIG:
            cmd[1] = 0x42U;
            cmd[2] = 0x6CU;
            *len = 4U;
            break;
        case ZDT_STEPPER_PARAM_STATE:
            cmd[1] = 0x43U;
            cmd[2] = 0x7AU;
            *len = 4U;
            break;
        default:
            return false;
    }

    cmd[*len - 1U] = ZDT_CHECK_BYTE;
    return true;
}

static uint8_t zdt_dlc_to_len(uint32_t dlc)
{
    if (dlc > 8U) {
        return 8U;
    }

    return (uint8_t) dlc;
}
