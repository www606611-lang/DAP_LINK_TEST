#include "boot_flash.h"
#include "boot_uart.h"
#include "crc32.h"
#include "firmware_layout.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define BOOT_PROTOCOL_MAGIC_0          0x55U
#define BOOT_PROTOCOL_MAGIC_1          0xAAU
#define BOOT_PROTOCOL_VERSION          1U
#define BOOT_PROTOCOL_HEADER_SIZE      6U
#define BOOT_PROTOCOL_CRC_SIZE         4U
#define BOOT_PROTOCOL_MAX_PAYLOAD      1028U
#define BOOT_PROTOCOL_DATA_SIZE        1024U
#define BOOT_RESPONSE_GUARD_CYCLES     (CPUCLK_FREQ / 200U)
#define BOOT_RESPONSE_SYNC_GAP_CYCLES  (CPUCLK_FREQ / 1000U)

#define BOOT_COMMAND_PING              0x01U
#define BOOT_COMMAND_BEGIN             0x02U
#define BOOT_COMMAND_DATA              0x03U
#define BOOT_COMMAND_END               0x04U
#define BOOT_COMMAND_RUN               0x05U

#define BOOT_STATUS_OK                 0U
#define BOOT_STATUS_BAD_VERSION        1U
#define BOOT_STATUS_BAD_LENGTH         2U
#define BOOT_STATUS_BAD_CRC            3U
#define BOOT_STATUS_BAD_STATE          4U
#define BOOT_STATUS_BAD_SEQUENCE       5U
#define BOOT_STATUS_BAD_OFFSET         6U
#define BOOT_STATUS_FLASH_ERROR        7U
#define BOOT_STATUS_IMAGE_ERROR        8U
#define BOOT_STATUS_UNKNOWN_COMMAND    9U

typedef enum {
    BOOT_RX_MAGIC_0 = 0,
    BOOT_RX_MAGIC_1,
    BOOT_RX_HEADER,
    BOOT_RX_PAYLOAD,
    BOOT_RX_CRC
} boot_rx_state_t;

typedef struct {
    boot_rx_state_t state;
    uint8_t header[BOOT_PROTOCOL_HEADER_SIZE];
    uint16_t header_index;
    uint16_t payload_length;
    uint16_t payload_index;
    uint16_t crc_index;
    uint8_t crc_bytes[BOOT_PROTOCOL_CRC_SIZE];
    uint32_t payload_words[(BOOT_PROTOCOL_MAX_PAYLOAD + 3U) / 4U];
} boot_rx_t;

static boot_rx_t g_rx;
static uint32_t g_program_words[BOOT_PROTOCOL_DATA_SIZE / 4U]
    __attribute__((aligned(8)));
static bool g_updating;
static uint32_t g_image_size;
static uint32_t g_expected_image_crc;
static uint32_t g_running_image_crc;
static uint32_t g_received_size;
static uint16_t g_expected_sequence;
static uint16_t g_last_data_sequence;
static uint32_t g_last_data_offset;
static uint16_t g_last_data_length;
static uint32_t g_last_data_frame_crc;
static bool g_have_last_data;

static void boot_force_motor_high_z(void);
static bool boot_mailbox_requested(void);
static bool boot_vectors_valid(uint32_t image_size);
static bool boot_image_valid(void);
static void boot_jump_to_application(void) __attribute__((noreturn));
static void boot_reset_receiver(void);
static void boot_receive_byte(uint8_t byte);
static void boot_process_frame(uint32_t frame_crc);
static void boot_send_response(uint8_t command, uint16_t sequence,
    uint8_t status, const uint8_t *data, uint16_t length);
static void boot_handle_begin(uint16_t sequence,
    const uint8_t *payload, uint16_t length);
static void boot_handle_data(uint16_t sequence,
    const uint8_t *payload, uint16_t length, uint32_t frame_crc);
static void boot_handle_end(uint16_t sequence, uint16_t length);
static bool boot_begin_update(uint32_t image_size, uint32_t image_crc);
static bool boot_finish_update(void);
static uint16_t boot_read_u16(const uint8_t *data);
static uint32_t boot_read_u32(const uint8_t *data);
static void boot_write_u16(uint8_t *data, uint16_t value);
static void boot_write_u32(uint8_t *data, uint32_t value);

int main(void)
{
    uint8_t byte;

    SYSCFG_DL_initPower();
    boot_force_motor_high_z();
    SYSCFG_DL_GPIO_init();
    boot_force_motor_high_z();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_BOOT_UART_init();
    BootUart_Init();

    if (!boot_mailbox_requested() && boot_image_valid()) {
        boot_jump_to_application();
    }

    boot_reset_receiver();
    while (1) {
        if (BootUart_TryRead(&byte)) {
            boot_receive_byte(byte);
        }
    }
}

static void boot_force_motor_high_z(void)
{
    const uint32_t motor_pins = DL_GPIO_PIN_23 | DL_GPIO_PIN_24 |
        DL_GPIO_PIN_29 | DL_GPIO_PIN_30;

    DL_GPIO_clearPins(GPIOA, motor_pins);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM53);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM54);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM4);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM5);
    DL_GPIO_enableOutput(GPIOA, motor_pins);
}

static bool boot_mailbox_requested(void)
{
    volatile firmware_mailbox_t *mailbox =
        (volatile firmware_mailbox_t *) FIRMWARE_MAILBOX_ADDRESS;
    bool requested = (mailbox->magic == FIRMWARE_MAILBOX_MAGIC) &&
        (mailbox->magic_inverse == FIRMWARE_MAILBOX_MAGIC_INVERSE);

    mailbox->magic = 0U;
    mailbox->magic_inverse = 0U;
    return requested;
}

static bool boot_vectors_valid(uint32_t image_size)
{
    const uint32_t *vectors =
        (const uint32_t *) FIRMWARE_APPLICATION_START;
    uint32_t stack_pointer = vectors[0];
    uint32_t reset_handler = vectors[1];
    uint32_t reset_address = reset_handler & ~1UL;
    uint32_t image_end = FIRMWARE_APPLICATION_START + image_size;

    return ((stack_pointer & 3U) == 0U) &&
        (stack_pointer >= FIRMWARE_SRAM_START) &&
        (stack_pointer <= FIRMWARE_MAILBOX_ADDRESS) &&
        ((reset_handler & 1U) != 0U) &&
        (reset_address >= FIRMWARE_APPLICATION_START) &&
        (reset_address < image_end);
}

static bool boot_image_valid(void)
{
    const firmware_metadata_t *metadata =
        (const firmware_metadata_t *) FIRMWARE_METADATA_ADDRESS;

    if ((metadata->begin_magic == UINT32_MAX) &&
        (metadata->complete_magic == UINT32_MAX)) {
        return boot_vectors_valid(FIRMWARE_APPLICATION_MAX_SIZE);
    }
    if ((metadata->begin_magic != FIRMWARE_METADATA_BEGIN_MAGIC) ||
        (metadata->version != FIRMWARE_METADATA_VERSION) ||
        (metadata->image_size == 0U) ||
        (metadata->image_size > FIRMWARE_APPLICATION_MAX_SIZE) ||
        (metadata->complete_magic != FIRMWARE_METADATA_COMPLETE_MAGIC) ||
        (metadata->complete_inverse !=
            ~FIRMWARE_METADATA_COMPLETE_MAGIC) ||
        !boot_vectors_valid(metadata->image_size)) {
        return false;
    }
    return Crc32_Calculate((const uint8_t *) FIRMWARE_APPLICATION_START,
        metadata->image_size) == metadata->image_crc32;
}

static void boot_jump_to_application(void)
{
    const uint32_t *vectors =
        (const uint32_t *) FIRMWARE_APPLICATION_START;
    void (*reset_handler)(void) = (void (*)(void)) vectors[1];

    __disable_irq();
    SCB->VTOR = FIRMWARE_APPLICATION_START;
    __DSB();
    __ISB();
    __set_MSP(vectors[0]);
    reset_handler();
    while (1) {
    }
}

static void boot_reset_receiver(void)
{
    g_rx.state = BOOT_RX_MAGIC_0;
    g_rx.header_index = 0U;
    g_rx.payload_length = 0U;
    g_rx.payload_index = 0U;
    g_rx.crc_index = 0U;
}

static void boot_receive_byte(uint8_t byte)
{
    uint8_t *payload = (uint8_t *) g_rx.payload_words;

    switch (g_rx.state) {
        case BOOT_RX_MAGIC_0:
            if (byte == BOOT_PROTOCOL_MAGIC_0) {
                g_rx.state = BOOT_RX_MAGIC_1;
            }
            break;
        case BOOT_RX_MAGIC_1:
            if (byte == BOOT_PROTOCOL_MAGIC_1) {
                g_rx.header_index = 0U;
                g_rx.state = BOOT_RX_HEADER;
            } else if (byte != BOOT_PROTOCOL_MAGIC_0) {
                g_rx.state = BOOT_RX_MAGIC_0;
            }
            break;
        case BOOT_RX_HEADER:
            g_rx.header[g_rx.header_index++] = byte;
            if (g_rx.header_index == BOOT_PROTOCOL_HEADER_SIZE) {
                g_rx.payload_length = boot_read_u16(&g_rx.header[4]);
                if (g_rx.payload_length > BOOT_PROTOCOL_MAX_PAYLOAD) {
                    boot_send_response(g_rx.header[1],
                        boot_read_u16(&g_rx.header[2]),
                        BOOT_STATUS_BAD_LENGTH, 0, 0U);
                    boot_reset_receiver();
                } else if (g_rx.payload_length == 0U) {
                    g_rx.crc_index = 0U;
                    g_rx.state = BOOT_RX_CRC;
                } else {
                    g_rx.payload_index = 0U;
                    g_rx.state = BOOT_RX_PAYLOAD;
                }
            }
            break;
        case BOOT_RX_PAYLOAD:
            payload[g_rx.payload_index++] = byte;
            if (g_rx.payload_index == g_rx.payload_length) {
                g_rx.crc_index = 0U;
                g_rx.state = BOOT_RX_CRC;
            }
            break;
        case BOOT_RX_CRC:
            g_rx.crc_bytes[g_rx.crc_index++] = byte;
            if (g_rx.crc_index == BOOT_PROTOCOL_CRC_SIZE) {
                uint32_t calculated = Crc32_Update(Crc32_Begin(),
                    g_rx.header, sizeof(g_rx.header));
                calculated = Crc32_End(Crc32_Update(calculated,
                    payload, g_rx.payload_length));
                if (calculated == boot_read_u32(g_rx.crc_bytes)) {
                    boot_process_frame(calculated);
                } else {
                    boot_send_response(g_rx.header[1],
                        boot_read_u16(&g_rx.header[2]),
                        BOOT_STATUS_BAD_CRC, 0, 0U);
                }
                boot_reset_receiver();
            }
            break;
        default:
            boot_reset_receiver();
            break;
    }
}

static void boot_process_frame(uint32_t frame_crc)
{
    uint8_t command = g_rx.header[1];
    uint16_t sequence = boot_read_u16(&g_rx.header[2]);
    const uint8_t *payload = (const uint8_t *) g_rx.payload_words;

    if (g_rx.header[0] != BOOT_PROTOCOL_VERSION) {
        boot_send_response(command, sequence, BOOT_STATUS_BAD_VERSION, 0, 0U);
        return;
    }

    switch (command) {
        case BOOT_COMMAND_PING: {
            uint8_t data[6];
            boot_write_u32(data, FIRMWARE_APPLICATION_MAX_SIZE);
            data[4] = boot_image_valid() ? 1U : 0U;
            data[5] = g_updating ? 1U : 0U;
            boot_send_response(command, sequence, BOOT_STATUS_OK,
                data, sizeof(data));
            break;
        }
        case BOOT_COMMAND_BEGIN:
            boot_handle_begin(sequence, payload, g_rx.payload_length);
            break;
        case BOOT_COMMAND_DATA:
            boot_handle_data(sequence, payload, g_rx.payload_length,
                frame_crc);
            break;
        case BOOT_COMMAND_END:
            boot_handle_end(sequence, g_rx.payload_length);
            break;
        case BOOT_COMMAND_RUN:
            if ((g_rx.payload_length != 0U) || !boot_image_valid()) {
                boot_send_response(command, sequence,
                    BOOT_STATUS_IMAGE_ERROR, 0, 0U);
            } else {
                boot_send_response(command, sequence, BOOT_STATUS_OK, 0, 0U);
                BootUart_WaitTxIdle();
                NVIC_SystemReset();
            }
            break;
        default:
            boot_send_response(command, sequence,
                BOOT_STATUS_UNKNOWN_COMMAND, 0, 0U);
            break;
    }
}

static void boot_send_response(uint8_t command, uint16_t sequence,
    uint8_t status, const uint8_t *data, uint16_t length)
{
    uint8_t sync[8] = {
        BOOT_PROTOCOL_MAGIC_0, BOOT_PROTOCOL_MAGIC_0,
        BOOT_PROTOCOL_MAGIC_0, BOOT_PROTOCOL_MAGIC_0,
        BOOT_PROTOCOL_MAGIC_0, BOOT_PROTOCOL_MAGIC_0,
        BOOT_PROTOCOL_MAGIC_0, BOOT_PROTOCOL_MAGIC_0
    };
    uint8_t sync_end = BOOT_PROTOCOL_MAGIC_1;
    uint8_t header[BOOT_PROTOCOL_HEADER_SIZE];
    uint8_t payload[16];
    uint8_t crc_bytes[4];
    uint16_t payload_length = (uint16_t) (length + 1U);
    uint16_t i;
    uint32_t crc;

    if (length > (sizeof(payload) - 1U)) {
        return;
    }
    header[0] = BOOT_PROTOCOL_VERSION;
    header[1] = command | 0x80U;
    boot_write_u16(&header[2], sequence);
    boot_write_u16(&header[4], payload_length);
    payload[0] = status;
    for (i = 0U; i < length; i++) {
        payload[i + 1U] = data[i];
    }
    crc = Crc32_Update(Crc32_Begin(), header, sizeof(header));
    crc = Crc32_End(Crc32_Update(crc, payload, payload_length));
    boot_write_u32(crc_bytes, crc);
    delay_cycles(BOOT_RESPONSE_GUARD_CYCLES);
    /* JDY-31 uplink can corrupt initial turnaround bytes; repeated 0x55
     * lets the host parser reacquire before the terminating 0xAA. */
    BootUart_Write(sync, sizeof(sync));
    delay_cycles(BOOT_RESPONSE_SYNC_GAP_CYCLES);
    BootUart_Write(&sync_end, sizeof(sync_end));
    delay_cycles(BOOT_RESPONSE_SYNC_GAP_CYCLES);
    BootUart_Write(header, sizeof(header));
    BootUart_Write(payload, payload_length);
    BootUart_Write(crc_bytes, sizeof(crc_bytes));
    BootUart_WaitTxIdle();
}

static void boot_handle_begin(uint16_t sequence,
    const uint8_t *payload, uint16_t length)
{
    uint32_t image_size;
    uint32_t image_crc;

    if (length != 8U) {
        boot_send_response(BOOT_COMMAND_BEGIN, sequence,
            BOOT_STATUS_BAD_LENGTH, 0, 0U);
        return;
    }
    image_size = boot_read_u32(payload);
    image_crc = boot_read_u32(payload + 4U);
    if (!boot_begin_update(image_size, image_crc)) {
        boot_send_response(BOOT_COMMAND_BEGIN, sequence,
            (image_size == 0U || image_size > FIRMWARE_APPLICATION_MAX_SIZE) ?
                BOOT_STATUS_IMAGE_ERROR : BOOT_STATUS_FLASH_ERROR,
            0, 0U);
        return;
    }
    g_expected_sequence = (uint16_t) (sequence + 1U);
    boot_send_response(BOOT_COMMAND_BEGIN, sequence, BOOT_STATUS_OK, 0, 0U);
}

static void boot_handle_data(uint16_t sequence,
    const uint8_t *payload, uint16_t length, uint32_t frame_crc)
{
    uint8_t response[4];
    uint32_t offset;
    uint16_t data_length;
    uint16_t i;
    uint16_t padded_length;

    if ((length < 5U) || (length > (BOOT_PROTOCOL_DATA_SIZE + 4U))) {
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_BAD_LENGTH, 0, 0U);
        return;
    }
    offset = boot_read_u32(payload);
    data_length = (uint16_t) (length - 4U);

    if (g_have_last_data && (sequence == g_last_data_sequence) &&
        (offset == g_last_data_offset) &&
        (data_length == g_last_data_length) &&
        (frame_crc == g_last_data_frame_crc)) {
        boot_write_u32(response, g_received_size);
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_OK, response, sizeof(response));
        return;
    }
    if (!g_updating) {
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_BAD_STATE, 0, 0U);
        return;
    }
    if (sequence != g_expected_sequence) {
        boot_write_u32(response, g_received_size);
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_BAD_SEQUENCE, response, sizeof(response));
        return;
    }
    if ((offset != g_received_size) || ((offset & 7U) != 0U) ||
        (data_length > (g_image_size - g_received_size))) {
        boot_write_u32(response, g_received_size);
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_BAD_OFFSET, response, sizeof(response));
        return;
    }

    padded_length = (uint16_t) ((data_length + 7U) & ~7U);
    for (i = 0U; i < padded_length; i++) {
        ((uint8_t *) g_program_words)[i] =
            (i < data_length) ? payload[i + 4U] : 0xFFU;
    }
    if (!BootFlash_Program(FIRMWARE_APPLICATION_START + offset,
            g_program_words, padded_length)) {
        boot_send_response(BOOT_COMMAND_DATA, sequence,
            BOOT_STATUS_FLASH_ERROR, 0, 0U);
        return;
    }

    g_running_image_crc = Crc32_Update(g_running_image_crc,
        payload + 4U, data_length);
    g_received_size += data_length;
    g_last_data_sequence = sequence;
    g_last_data_offset = offset;
    g_last_data_length = data_length;
    g_last_data_frame_crc = frame_crc;
    g_have_last_data = true;
    g_expected_sequence = (uint16_t) (sequence + 1U);
    boot_write_u32(response, g_received_size);
    boot_send_response(BOOT_COMMAND_DATA, sequence,
        BOOT_STATUS_OK, response, sizeof(response));
}

static void boot_handle_end(uint16_t sequence, uint16_t length)
{
    if (length != 0U) {
        boot_send_response(BOOT_COMMAND_END, sequence,
            BOOT_STATUS_BAD_LENGTH, 0, 0U);
        return;
    }
    if (!g_updating) {
        boot_send_response(BOOT_COMMAND_END, sequence,
            boot_image_valid() ? BOOT_STATUS_OK : BOOT_STATUS_BAD_STATE,
            0, 0U);
        return;
    }
    if (sequence != g_expected_sequence) {
        boot_send_response(BOOT_COMMAND_END, sequence,
            BOOT_STATUS_BAD_SEQUENCE, 0, 0U);
        return;
    }
    if (!boot_finish_update()) {
        boot_send_response(BOOT_COMMAND_END, sequence,
            BOOT_STATUS_IMAGE_ERROR, 0, 0U);
        return;
    }
    g_updating = false;
    boot_send_response(BOOT_COMMAND_END, sequence, BOOT_STATUS_OK, 0, 0U);
}

static bool boot_begin_update(uint32_t image_size, uint32_t image_crc)
{
    firmware_metadata_t metadata;
    uint32_t address;
    uint32_t erase_end;

    if ((image_size == 0U) ||
        (image_size > FIRMWARE_APPLICATION_MAX_SIZE)) {
        return false;
    }

    if (!BootFlash_EraseSector(FIRMWARE_METADATA_ADDRESS)) {
        return false;
    }
    metadata.begin_magic = FIRMWARE_METADATA_BEGIN_MAGIC;
    metadata.version = FIRMWARE_METADATA_VERSION;
    metadata.image_size = image_size;
    metadata.image_crc32 = image_crc;
    metadata.complete_magic = UINT32_MAX;
    metadata.complete_inverse = UINT32_MAX;
    if (!BootFlash_Program(FIRMWARE_METADATA_ADDRESS,
            (const uint32_t *) &metadata, 16U)) {
        return false;
    }

    erase_end = FIRMWARE_APPLICATION_START + image_size;
    for (address = FIRMWARE_APPLICATION_START;
         address < erase_end; address += DL_FLASHCTL_SECTOR_SIZE) {
        if (!BootFlash_EraseSector(address)) {
            return false;
        }
    }

    g_updating = true;
    g_image_size = image_size;
    g_expected_image_crc = image_crc;
    g_running_image_crc = Crc32_Begin();
    g_received_size = 0U;
    g_have_last_data = false;
    return true;
}

static bool boot_finish_update(void)
{
    uint32_t complete_words[2];
    uint32_t calculated_crc;

    if (g_received_size != g_image_size) {
        return false;
    }
    calculated_crc = Crc32_End(g_running_image_crc);
    if ((calculated_crc != g_expected_image_crc) ||
        (Crc32_Calculate((const uint8_t *) FIRMWARE_APPLICATION_START,
            g_image_size) != g_expected_image_crc) ||
        !boot_vectors_valid(g_image_size)) {
        return false;
    }
    complete_words[0] = FIRMWARE_METADATA_COMPLETE_MAGIC;
    complete_words[1] = ~FIRMWARE_METADATA_COMPLETE_MAGIC;
    return BootFlash_Program(FIRMWARE_METADATA_ADDRESS + 16U,
        complete_words, sizeof(complete_words));
}

static uint16_t boot_read_u16(const uint8_t *data)
{
    return (uint16_t) data[0] | ((uint16_t) data[1] << 8U);
}

static uint32_t boot_read_u32(const uint8_t *data)
{
    return (uint32_t) data[0] | ((uint32_t) data[1] << 8U) |
        ((uint32_t) data[2] << 16U) | ((uint32_t) data[3] << 24U);
}

static void boot_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
}

static void boot_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
    data[2] = (uint8_t) (value >> 16U);
    data[3] = (uint8_t) (value >> 24U);
}
