#include "pid_tuning_store.h"

#include "encoder.h"
#include "encoder_position_control.h"
#include "encoder_speed_control.h"
#include "line_tracking_control.h"
#include "ti_msp_dl_config.h"
#include "yaw_angle_control.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ti/driverlib/dl_flashctl.h>
#include <ti/driverlib/m0p/dl_core.h>

#define PID_TUNING_STORE_MAGIC       0x50494431UL
#define PID_TUNING_STORE_VERSION     3UL
#define PID_TUNING_STORE_ADDRESS     0x0001FC00UL
#define PID_TUNING_STORE_SECTOR_SIZE 1024UL

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float feedforward_pwm;
    float feedforward_reference_pps;
    float forward_min_drive_pwm;
    float reverse_min_drive_pwm;
    float min_drive_reference_pps;
} pid_store_speed_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
} pid_store_pid_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float min_turn_speed_pps;
    float max_turn_speed_pps;
} pid_store_yaw_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float base_speed_pps;
    float left_pwm_limit;
    float right_pwm_limit;
} pid_store_line_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t crc32;
    pid_store_speed_t speed_left;
    pid_store_speed_t speed_right;
    pid_store_pid_t position_left_pos;
    pid_store_pid_t position_right_pos;
    pid_store_pid_t position_left_spd;
    pid_store_pid_t position_right_spd;
    pid_store_yaw_t yaw;
    pid_store_line_t line;
    uint32_t reserved;
} pid_store_image_t;

typedef union {
    pid_store_image_t image;
    uint32_t word[(sizeof(pid_store_image_t) + sizeof(uint32_t) - 1U) /
        sizeof(uint32_t)];
} pid_store_flash_words_t;

typedef char pid_store_image_size_must_align_to_flash_write[
    ((sizeof(pid_store_image_t) % sizeof(uint64_t)) == 0U) ? 1 : -1];

static pid_tuning_store_status_t g_pid_tuning_store_status =
    PID_TUNING_STORE_NOT_FOUND;

static void pid_tuning_store_capture(pid_store_image_t *image);
static void pid_tuning_store_apply(const pid_store_image_t *image);
static bool pid_tuning_store_validate(const pid_store_image_t *image);
static uint32_t pid_tuning_store_crc32(
    const uint8_t *data, uint32_t length);
static bool pid_tuning_store_write_image(const pid_store_image_t *image);
static bool pid_tuning_store_erase_sector(void);
static bool pid_tuning_store_program_words(
    const uint32_t *data, uint32_t word_count);
static bool pid_tuning_store_is_blank(const pid_store_image_t *image);

pid_tuning_store_status_t PidTuningStore_SaveCurrent(void)
{
    pid_store_image_t image;

    pid_tuning_store_capture(&image);
    image.magic = PID_TUNING_STORE_MAGIC;
    image.version = PID_TUNING_STORE_VERSION;
    image.size = (uint32_t) sizeof(image);
    image.crc32 = 0UL;
    image.crc32 = pid_tuning_store_crc32(
        (const uint8_t *) &image, (uint32_t) sizeof(image));

    if (!pid_tuning_store_write_image(&image)) {
        g_pid_tuning_store_status = PID_TUNING_STORE_FLASH_ERROR;
        return g_pid_tuning_store_status;
    }

    g_pid_tuning_store_status = PID_TUNING_STORE_OK;
    return g_pid_tuning_store_status;
}

pid_tuning_store_status_t PidTuningStore_LoadApply(void)
{
    const pid_store_image_t *image =
        (const pid_store_image_t *) PID_TUNING_STORE_ADDRESS;

    if (pid_tuning_store_is_blank(image)) {
        g_pid_tuning_store_status = PID_TUNING_STORE_NOT_FOUND;
        return g_pid_tuning_store_status;
    }

    if ((image->magic != PID_TUNING_STORE_MAGIC) ||
        (image->version != PID_TUNING_STORE_VERSION)) {
        g_pid_tuning_store_status = PID_TUNING_STORE_NOT_FOUND;
        return g_pid_tuning_store_status;
    }

    if (image->size != (uint32_t) sizeof(pid_store_image_t)) {
        g_pid_tuning_store_status = PID_TUNING_STORE_BAD_SIZE;
        return g_pid_tuning_store_status;
    }

    if (!pid_tuning_store_validate(image)) {
        g_pid_tuning_store_status = PID_TUNING_STORE_BAD_CRC;
        return g_pid_tuning_store_status;
    }

    pid_tuning_store_apply(image);
    g_pid_tuning_store_status = PID_TUNING_STORE_OK;
    return g_pid_tuning_store_status;
}

pid_tuning_store_status_t PidTuningStore_Clear(void)
{
    if (!pid_tuning_store_erase_sector()) {
        g_pid_tuning_store_status = PID_TUNING_STORE_FLASH_ERROR;
        return g_pid_tuning_store_status;
    }

    g_pid_tuning_store_status = PID_TUNING_STORE_NOT_FOUND;
    return PID_TUNING_STORE_OK;
}

pid_tuning_store_status_t PidTuningStore_GetStatus(void)
{
    return g_pid_tuning_store_status;
}

const char *PidTuningStore_StatusText(pid_tuning_store_status_t status)
{
    switch (status) {
        case PID_TUNING_STORE_OK:
            return "ok";
        case PID_TUNING_STORE_NOT_FOUND:
            return "empty";
        case PID_TUNING_STORE_BAD_CRC:
            return "bad_crc";
        case PID_TUNING_STORE_BAD_SIZE:
            return "bad_size";
        case PID_TUNING_STORE_FLASH_ERROR:
            return "flash_err";
        default:
            return "unknown";
    }
}

bool PidTuningStore_HasValidImage(void)
{
    const pid_store_image_t *image =
        (const pid_store_image_t *) PID_TUNING_STORE_ADDRESS;

    return pid_tuning_store_validate(image);
}

static void pid_tuning_store_capture(pid_store_image_t *image)
{
    encoder_speed_control_config_t speed_config;
    encoder_position_control_pid_config_t position_config;
    yaw_angle_control_config_t yaw_config;
    line_tracking_config_t line_config;

    if (image == NULL) {
        return;
    }
    (void) memset(image, 0, sizeof(*image));

    EncoderSpeedControl_GetSpeedConfig(ENCODER_LEFT, &speed_config);
    image->speed_left.kp = speed_config.kp;
    image->speed_left.ki = speed_config.ki;
    image->speed_left.kd = speed_config.kd;
    image->speed_left.output_min = speed_config.output_min;
    image->speed_left.output_max = speed_config.output_max;
    image->speed_left.integral_min = speed_config.integral_min;
    image->speed_left.integral_max = speed_config.integral_max;
    image->speed_left.deadband = speed_config.deadband;
    image->speed_left.feedforward_pwm = speed_config.feedforward_pwm;
    image->speed_left.feedforward_reference_pps =
        speed_config.feedforward_reference_pps;
    image->speed_left.forward_min_drive_pwm =
        speed_config.forward_min_drive_pwm;
    image->speed_left.reverse_min_drive_pwm =
        speed_config.reverse_min_drive_pwm;
    image->speed_left.min_drive_reference_pps =
        speed_config.min_drive_reference_pps;

    EncoderSpeedControl_GetSpeedConfig(ENCODER_RIGHT, &speed_config);
    image->speed_right.kp = speed_config.kp;
    image->speed_right.ki = speed_config.ki;
    image->speed_right.kd = speed_config.kd;
    image->speed_right.output_min = speed_config.output_min;
    image->speed_right.output_max = speed_config.output_max;
    image->speed_right.integral_min = speed_config.integral_min;
    image->speed_right.integral_max = speed_config.integral_max;
    image->speed_right.deadband = speed_config.deadband;
    image->speed_right.feedforward_pwm = speed_config.feedforward_pwm;
    image->speed_right.feedforward_reference_pps =
        speed_config.feedforward_reference_pps;
    image->speed_right.forward_min_drive_pwm =
        speed_config.forward_min_drive_pwm;
    image->speed_right.reverse_min_drive_pwm =
        speed_config.reverse_min_drive_pwm;
    image->speed_right.min_drive_reference_pps =
        speed_config.min_drive_reference_pps;

    EncoderPositionControl_GetPositionConfig(ENCODER_LEFT, &position_config);
    image->position_left_pos.kp = position_config.kp;
    image->position_left_pos.ki = position_config.ki;
    image->position_left_pos.kd = position_config.kd;
    image->position_left_pos.output_min = position_config.output_min;
    image->position_left_pos.output_max = position_config.output_max;
    image->position_left_pos.integral_min = position_config.integral_min;
    image->position_left_pos.integral_max = position_config.integral_max;
    image->position_left_pos.deadband = position_config.deadband;

    EncoderPositionControl_GetPositionConfig(ENCODER_RIGHT, &position_config);
    image->position_right_pos.kp = position_config.kp;
    image->position_right_pos.ki = position_config.ki;
    image->position_right_pos.kd = position_config.kd;
    image->position_right_pos.output_min = position_config.output_min;
    image->position_right_pos.output_max = position_config.output_max;
    image->position_right_pos.integral_min = position_config.integral_min;
    image->position_right_pos.integral_max = position_config.integral_max;
    image->position_right_pos.deadband = position_config.deadband;

    EncoderPositionControl_GetSpeedConfig(ENCODER_LEFT, &position_config);
    image->position_left_spd.kp = position_config.kp;
    image->position_left_spd.ki = position_config.ki;
    image->position_left_spd.kd = position_config.kd;
    image->position_left_spd.output_min = position_config.output_min;
    image->position_left_spd.output_max = position_config.output_max;
    image->position_left_spd.integral_min = position_config.integral_min;
    image->position_left_spd.integral_max = position_config.integral_max;
    image->position_left_spd.deadband = position_config.deadband;

    EncoderPositionControl_GetSpeedConfig(ENCODER_RIGHT, &position_config);
    image->position_right_spd.kp = position_config.kp;
    image->position_right_spd.ki = position_config.ki;
    image->position_right_spd.kd = position_config.kd;
    image->position_right_spd.output_min = position_config.output_min;
    image->position_right_spd.output_max = position_config.output_max;
    image->position_right_spd.integral_min = position_config.integral_min;
    image->position_right_spd.integral_max = position_config.integral_max;
    image->position_right_spd.deadband = position_config.deadband;

    YawAngleControl_GetConfig(&yaw_config);
    image->yaw.kp = yaw_config.kp;
    image->yaw.ki = yaw_config.ki;
    image->yaw.kd = yaw_config.kd;
    image->yaw.output_min = yaw_config.output_min;
    image->yaw.output_max = yaw_config.output_max;
    image->yaw.integral_min = yaw_config.integral_min;
    image->yaw.integral_max = yaw_config.integral_max;
    image->yaw.deadband = yaw_config.deadband;
    image->yaw.min_turn_speed_pps = yaw_config.min_turn_speed_pps;
    image->yaw.max_turn_speed_pps = yaw_config.max_turn_speed_pps;

    LineTrackingControl_GetConfig(&line_config);
    image->line.kp = line_config.kp;
    image->line.ki = line_config.ki;
    image->line.kd = line_config.kd;
    image->line.output_min = line_config.output_min;
    image->line.output_max = line_config.output_max;
    image->line.integral_min = line_config.integral_min;
    image->line.integral_max = line_config.integral_max;
    image->line.deadband = line_config.deadband;
    image->line.base_speed_pps = line_config.base_speed_pps;
    image->line.left_pwm_limit = line_config.left_pwm_limit;
    image->line.right_pwm_limit = line_config.right_pwm_limit;
}

static void pid_tuning_store_apply(const pid_store_image_t *image)
{
    if (image == NULL) {
        return;
    }

    EncoderSpeedControl_SetSpeedTunings(ENCODER_LEFT,
        image->speed_left.kp, image->speed_left.ki, image->speed_left.kd);
    EncoderSpeedControl_SetSpeedOutputLimits(ENCODER_LEFT,
        image->speed_left.output_min, image->speed_left.output_max);
    EncoderSpeedControl_SetSpeedIntegralLimits(ENCODER_LEFT,
        image->speed_left.integral_min, image->speed_left.integral_max);
    EncoderSpeedControl_SetSpeedDeadband(
        ENCODER_LEFT, image->speed_left.deadband);
    EncoderSpeedControl_SetSpeedFeedforwardPwm(
        ENCODER_LEFT, image->speed_left.feedforward_pwm);
    EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
        ENCODER_LEFT, image->speed_left.feedforward_reference_pps);
    EncoderSpeedControl_SetSpeedMinDriveConfig(ENCODER_LEFT,
        image->speed_left.forward_min_drive_pwm,
        image->speed_left.reverse_min_drive_pwm,
        image->speed_left.min_drive_reference_pps);

    EncoderSpeedControl_SetSpeedTunings(ENCODER_RIGHT,
        image->speed_right.kp, image->speed_right.ki, image->speed_right.kd);
    EncoderSpeedControl_SetSpeedOutputLimits(ENCODER_RIGHT,
        image->speed_right.output_min, image->speed_right.output_max);
    EncoderSpeedControl_SetSpeedIntegralLimits(ENCODER_RIGHT,
        image->speed_right.integral_min, image->speed_right.integral_max);
    EncoderSpeedControl_SetSpeedDeadband(
        ENCODER_RIGHT, image->speed_right.deadband);
    EncoderSpeedControl_SetSpeedFeedforwardPwm(
        ENCODER_RIGHT, image->speed_right.feedforward_pwm);
    EncoderSpeedControl_SetSpeedFeedforwardReferencePps(
        ENCODER_RIGHT, image->speed_right.feedforward_reference_pps);
    EncoderSpeedControl_SetSpeedMinDriveConfig(ENCODER_RIGHT,
        image->speed_right.forward_min_drive_pwm,
        image->speed_right.reverse_min_drive_pwm,
        image->speed_right.min_drive_reference_pps);

    EncoderPositionControl_SetPositionTunings(ENCODER_LEFT,
        image->position_left_pos.kp, image->position_left_pos.ki,
        image->position_left_pos.kd);
    EncoderPositionControl_SetPositionOutputLimits(ENCODER_LEFT,
        image->position_left_pos.output_min,
        image->position_left_pos.output_max);
    EncoderPositionControl_SetPositionIntegralLimits(ENCODER_LEFT,
        image->position_left_pos.integral_min,
        image->position_left_pos.integral_max);
    EncoderPositionControl_SetPositionDeadband(ENCODER_LEFT,
        image->position_left_pos.deadband);

    EncoderPositionControl_SetPositionTunings(ENCODER_RIGHT,
        image->position_right_pos.kp, image->position_right_pos.ki,
        image->position_right_pos.kd);
    EncoderPositionControl_SetPositionOutputLimits(ENCODER_RIGHT,
        image->position_right_pos.output_min,
        image->position_right_pos.output_max);
    EncoderPositionControl_SetPositionIntegralLimits(ENCODER_RIGHT,
        image->position_right_pos.integral_min,
        image->position_right_pos.integral_max);
    EncoderPositionControl_SetPositionDeadband(ENCODER_RIGHT,
        image->position_right_pos.deadband);

    EncoderPositionControl_SetSpeedTunings(ENCODER_LEFT,
        image->position_left_spd.kp, image->position_left_spd.ki,
        image->position_left_spd.kd);
    EncoderPositionControl_SetSpeedOutputLimits(ENCODER_LEFT,
        image->position_left_spd.output_min,
        image->position_left_spd.output_max);
    EncoderPositionControl_SetSpeedIntegralLimits(ENCODER_LEFT,
        image->position_left_spd.integral_min,
        image->position_left_spd.integral_max);
    EncoderPositionControl_SetSpeedDeadband(ENCODER_LEFT,
        image->position_left_spd.deadband);

    EncoderPositionControl_SetSpeedTunings(ENCODER_RIGHT,
        image->position_right_spd.kp, image->position_right_spd.ki,
        image->position_right_spd.kd);
    EncoderPositionControl_SetSpeedOutputLimits(ENCODER_RIGHT,
        image->position_right_spd.output_min,
        image->position_right_spd.output_max);
    EncoderPositionControl_SetSpeedIntegralLimits(ENCODER_RIGHT,
        image->position_right_spd.integral_min,
        image->position_right_spd.integral_max);
    EncoderPositionControl_SetSpeedDeadband(ENCODER_RIGHT,
        image->position_right_spd.deadband);

    YawAngleControl_SetTunings(image->yaw.kp, image->yaw.ki, image->yaw.kd);
    YawAngleControl_SetOutputLimits(
        image->yaw.output_min, image->yaw.output_max);
    YawAngleControl_SetIntegralLimits(
        image->yaw.integral_min, image->yaw.integral_max);
    YawAngleControl_SetDeadband(image->yaw.deadband);
    YawAngleControl_SetMinTurnSpeedPps(image->yaw.min_turn_speed_pps);
    YawAngleControl_SetMaxTurnSpeedPps(image->yaw.max_turn_speed_pps);

    LineTrackingControl_SetTunings(
        image->line.kp, image->line.ki, image->line.kd);
    LineTrackingControl_SetOutputLimits(
        image->line.output_min, image->line.output_max);
    LineTrackingControl_SetIntegralLimits(
        image->line.integral_min, image->line.integral_max);
    LineTrackingControl_SetDeadband(image->line.deadband);
    LineTrackingControl_SetBaseSpeedPps(image->line.base_speed_pps);
    LineTrackingControl_SetDriveOutputLimits(
        image->line.left_pwm_limit, image->line.right_pwm_limit);

    EncoderPositionControl_SyncSpeedFromCurrent();
}

static bool pid_tuning_store_validate(const pid_store_image_t *image)
{
    pid_store_image_t copy;
    uint32_t expected_crc;

    if ((image == NULL) ||
        (image->magic != PID_TUNING_STORE_MAGIC) ||
        (image->version != PID_TUNING_STORE_VERSION) ||
        (image->size != (uint32_t) sizeof(pid_store_image_t))) {
        return false;
    }

    (void) memcpy(&copy, image, sizeof(copy));
    expected_crc = copy.crc32;
    copy.crc32 = 0UL;

    return pid_tuning_store_crc32(
        (const uint8_t *) &copy, (uint32_t) sizeof(copy)) == expected_crc;
}

static uint32_t pid_tuning_store_crc32(
    const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;

    if (data == NULL) {
        return 0UL;
    }

    for (i = 0UL; i < length; i++) {
        crc ^= (uint32_t) data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return ~crc;
}

static bool pid_tuning_store_write_image(const pid_store_image_t *image)
{
    pid_store_flash_words_t verify;
    pid_store_flash_words_t write_words;
    uint32_t word_count;

    if (image == NULL) {
        return false;
    }
    if (sizeof(pid_store_image_t) > PID_TUNING_STORE_SECTOR_SIZE) {
        return false;
    }
    if ((sizeof(pid_store_image_t) % sizeof(uint64_t)) != 0U) {
        return false;
    }

    (void) memset(&write_words, 0xFF, sizeof(write_words));
    (void) memcpy(&write_words.image, image, sizeof(*image));
    word_count = (uint32_t) (sizeof(write_words.word) /
        sizeof(write_words.word[0]));

    __disable_irq();
    if (!pid_tuning_store_erase_sector()) {
        __enable_irq();
        return false;
    }
    if (!pid_tuning_store_program_words(write_words.word, word_count)) {
        __enable_irq();
        return false;
    }
    __enable_irq();

    (void) memcpy(&verify,
        (const void *) PID_TUNING_STORE_ADDRESS, sizeof(verify));
    return memcmp(&verify.image, &write_words.image,
        sizeof(pid_store_image_t)) == 0;
}

static bool pid_tuning_store_erase_sector(void)
{
    DL_FLASHCTL_COMMAND_STATUS status;

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, PID_TUNING_STORE_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        PID_TUNING_STORE_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    return status == DL_FLASHCTL_COMMAND_STATUS_PASSED;
}

static bool pid_tuning_store_program_words(
    const uint32_t *data, uint32_t word_count)
{
    uint32_t i;
    uint32_t address = PID_TUNING_STORE_ADDRESS;

    if (data == NULL) {
        return false;
    }
    if ((word_count == 0U) || ((word_count % 2U) != 0U)) {
        return false;
    }

    for (i = 0UL; i < word_count; i += 2U) {
        DL_FLASHCTL_COMMAND_STATUS status;

        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, address,
            DL_FLASHCTL_REGION_SELECT_MAIN);
        status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, address, &data[i]);
        if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
            return false;
        }
        address += sizeof(uint64_t);
    }

    return true;
}

static bool pid_tuning_store_is_blank(const pid_store_image_t *image)
{
    const uint32_t *words = (const uint32_t *) image;
    uint32_t count = (uint32_t) (sizeof(*image) / sizeof(uint32_t));
    uint32_t i;

    if (image == NULL) {
        return true;
    }

    for (i = 0UL; i < count; i++) {
        if (words[i] != 0xFFFFFFFFUL) {
            return false;
        }
    }

    return true;
}
