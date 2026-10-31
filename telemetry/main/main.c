
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

#define TAG                     "TELEMETRY"

/* CAN / TWAI */
#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4
#define CAN_BITRATE             500000

#define CAN_QUEUE_LEN           64
#define CAN_FRAME_LENGTH        8

#define TELEMETRY_ID_MIN        0x500
#define TELEMETRY_ID_MAX        0x509

/* LoRa RYLR993 */
#define LORA_UART               UART_NUM_2
#define LORA_TX_GPIO             GPIO_NUM_17
#define LORA_RX_GPIO             GPIO_NUM_16
#define LORA_BAUD               9600

#define PILOT_ADDRESS           2

/* Telemetry batching */
#define BATCH_MAX                4
#define BATCH_WAIT_MS            50

/* LoRa communication */
#define RADIO_RESPONSE_TIMEOUT_MS 200
#define RADIO_PAYLOAD_MAX        240

/* Monitoring */
#define STATUS_INTERVAL_MS       5000

/* ============================================================
 * DATA STRUCTURES
 * ============================================================ */

typedef struct {
    uint32_t id;
    uint8_t data[CAN_FRAME_LENGTH];
} can_msg_t;

typedef struct {
    uint32_t can_received;
    uint32_t queue_drops;
    uint32_t radio_success;
    uint32_t radio_failures;
    uint32_t invalid_frames;
} telemetry_stats_t;

/* ============================================================
 * GLOBAL VARIABLES
 * ============================================================ */

static QueueHandle_t can_queue = NULL;
static twai_node_handle_t can_node = NULL;

static telemetry_stats_t stats = {0};

static uint32_t radio_sequence = 0;

/* ============================================================
 * CAN UTILITIES
 * ============================================================ */

static bool is_telemetry_id(uint32_t id)
{
    return (id >= TELEMETRY_ID_MIN &&
            id <= TELEMETRY_ID_MAX);
}

/* ============================================================
 * CAN RECEIVE CALLBACK
 *
 * Runs in ISR context.
 * Keep processing minimal.
 * ============================================================ */

static bool can_rx_callback(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx)
{
    (void)edata;
    (void)user_ctx;

    can_msg_t message = {0};

    twai_frame_t frame = {
        .buffer = message.data,
        .buffer_len = sizeof(message.data),
    };

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
        return false;
    }

    /* Accept only our telemetry ID range. */
    if (!is_telemetry_id(frame.header.id)) {
        return false;
    }

    /* Reject malformed payload lengths. */
    if (frame.header.dlc != CAN_FRAME_LENGTH) {
        stats.invalid_frames++;
        return false;
    }

    message.id = frame.header.id;

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xQueueSendFromISR(
            can_queue,
            &message,
            &higher_priority_task_woken) != pdTRUE) {

        stats.queue_drops++;
    } else {
        stats.can_received++;
    }

    return higher_priority_task_woken == pdTRUE;
}

/* ============================================================
 * CAN INITIALIZATION
 * ============================================================ */

static void can_init(void)
{
    can_queue = xQueueCreate(
        CAN_QUEUE_LEN,
        sizeof(can_msg_t));

    if (can_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN queue");
        abort();
    }

    twai_onchip_node_config_t config = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 4,
    };

    ESP_ERROR_CHECK(
        twai_new_node_onchip(&config, &can_node));

    twai_event_callbacks_t callbacks = {
        .on_rx_done = can_rx_callback,
    };

    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(
            can_node,
            &callbacks,
            NULL));

    ESP_ERROR_CHECK(
        twai_node_enable(can_node));

    ESP_LOGI(TAG, "CAN initialized");
    ESP_LOGI(TAG, "Bitrate : %lu bit/s",
             (unsigned long)CAN_BITRATE);
    ESP_LOGI(TAG, "TX GPIO : %d", CAN_TX_GPIO);
    ESP_LOGI(TAG, "RX GPIO : %d", CAN_RX_GPIO);

    ESP_LOGI(TAG,
             "Listening for telemetry IDs 0x%03X-0x%03X",
             TELEMETRY_ID_MIN,
             TELEMETRY_ID_MAX);
}

/* ============================================================
 * LORA UART INITIALIZATION
 * ============================================================ */

static void lora_uart_init(void)
{
    uart_config_t config = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(
        uart_driver_install(
            LORA_UART,
            2048,
            2048,
            0,
            NULL,
            0));

    ESP_ERROR_CHECK(
        uart_param_config(
            LORA_UART,
            &config));

    ESP_ERROR_CHECK(
        uart_set_pin(
            LORA_UART,
            LORA_TX_GPIO,
            LORA_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "LoRa UART initialized");
    ESP_LOGI(TAG, "UART  : UART2");
    ESP_LOGI(TAG, "Baud  : %d", LORA_BAUD);
    ESP_LOGI(TAG, "TX    : GPIO %d", LORA_TX_GPIO);
    ESP_LOGI(TAG, "RX    : GPIO %d", LORA_RX_GPIO);
}

/* ============================================================
 * CRC-16 / CCITT-FALSE
 *
 * Polynomial : 0x1021
 * Initial   : 0xFFFF
 * XOR Out   : 0x0000
 * ============================================================ */

static uint16_t crc16_ccitt(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {

        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {

            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* ============================================================
 * HEX ENCODING
 * ============================================================ */

static void bytes_to_hex(
    char *output,
    const uint8_t *data,
    size_t length)
{
    static const char digits[] =
        "0123456789ABCDEF";

    for (size_t i = 0; i < length; i++) {

        output[2 * i] =
            digits[(data[i] >> 4) & 0x0F];

        output[2 * i + 1] =
            digits[data[i] & 0x0F];
    }

    output[2 * length] = '\0';
}

/* ============================================================
 * LORA BATCH TRANSMISSION
 *
 * Format:
 *
 * LM1,<sequence>,<count>,
 * <CAN_ID><16_HEX_DATA>,
 * ...
 * *<CRC16>
 *
 * ============================================================ */

static bool lora_send_batch(
    const can_msg_t *messages,
    unsigned count)
{
    if (messages == NULL ||
        count == 0 ||
        count > BATCH_MAX) {

        ESP_LOGE(TAG, "Invalid telemetry batch");
        return false;
    }

    char payload[RADIO_PAYLOAD_MAX + 1];

    int used = snprintf(
        payload,
        sizeof(payload),
        "LM1,%" PRIu32 ",%u",
        ++radio_sequence,
        count);

    if (used < 0 ||
        used >= sizeof(payload)) {

        return false;
    }

    /* Append individual CAN frames. */
    for (unsigned i = 0; i < count; i++) {

        char hex[CAN_FRAME_LENGTH * 2 + 1];

        bytes_to_hex(
            hex,
            messages[i].data,
            CAN_FRAME_LENGTH);

        int written = snprintf(
            payload + used,
            sizeof(payload) - (size_t)used,
            ",%03" PRIX32 "%s",
            messages[i].id,
            hex);

        if (written < 0 ||
            written >=
            (int)(sizeof(payload) - (size_t)used)) {

            ESP_LOGE(TAG, "Payload buffer overflow");
            return false;
        }

        used += written;
    }

    /* Append CRC. */
    uint16_t crc = crc16_ccitt(
        (const uint8_t *)payload,
        (size_t)used);

    int written = snprintf(
        payload + used,
        sizeof(payload) - (size_t)used,
        "*%04X",
        crc);

    if (written < 0 ||
        written >=
        (int)(sizeof(payload) - (size_t)used)) {

        return false;
    }

    used += written;

    /* Construct RYLR993 AT command. */
    char command[RADIO_PAYLOAD_MAX + 40];

    int command_len = snprintf(
        command,
        sizeof(command),
        "AT+SEND=%d,%d,%s\r\n",
        PILOT_ADDRESS,
        used,
        payload);

    if (command_len <= 0 ||
        command_len >= sizeof(command)) {

        ESP_LOGE(TAG, "LoRa command overflow");
        return false;
    }

    /* Transmit. */
    uart_flush_input(LORA_UART);

    int sent = uart_write_bytes(
        LORA_UART,
        command,
        command_len);

    if (sent != command_len) {

        stats.radio_failures++;

        ESP_LOGE(TAG, "LoRa UART write failed");
        return false;
    }

    /* Wait for module response. */
    char response[160] = {0};

    int received = uart_read_bytes(
        LORA_UART,
        (uint8_t *)response,
        sizeof(response) - 1,
        pdMS_TO_TICKS(
            RADIO_RESPONSE_TIMEOUT_MS));

    if (received > 0) {
        response[received] = '\0';
    }

    bool success = (
        received > 0 &&
        (strstr(response, "+OK") != NULL ||
         strstr(response, "OK") != NULL));

    if (success) {
        stats.radio_success++;
    } else {
        stats.radio_failures++;
    }

    ESP_LOGI(TAG,
             "LoRa seq=%" PRIu32
             " frames=%u CRC=0x%04X status=%s",
             radio_sequence,
             count,
             crc,
             success ? "OK" : "FAILED");

    return success;
}

/* ============================================================
 * TELEMETRY TASK
 *
 * CAN Queue -> Batch -> CRC -> LoRa
 * ============================================================ */

static void telemetry_task(void *arg)
{
    (void)arg;

    can_msg_t batch[BATCH_MAX];

    TickType_t last_status =
        xTaskGetTickCount();

    while (1) {

        unsigned count = 0;

        /* Wait for first frame. */
        if (xQueueReceive(
                can_queue,
                &batch[count],
                pdMS_TO_TICKS(BATCH_WAIT_MS))
                == pdTRUE) {

            count = 1;

            /* Collect additional frames without blocking. */
            while (count < BATCH_MAX &&
                   xQueueReceive(
                       can_queue,
                       &batch[count],
                       0) == pdTRUE) {

                count++;
            }

            /* Send batch. */
            lora_send_batch(batch, count);
        }

        /* Periodic system statistics. */
        if ((xTaskGetTickCount() - last_status) >=
            pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {

            ESP_LOGI(TAG,
                     "STAT | CAN RX=%" PRIu32
                     " | Queue drops=%" PRIu32
                     " | Invalid=%" PRIu32
                     " | LoRa OK=%" PRIu32
                     " | LoRa FAIL=%" PRIu32,
                     stats.can_received,
                     stats.queue_drops,
                     stats.invalid_frames,
                     stats.radio_success,
                     stats.radio_failures);

            last_status = xTaskGetTickCount();
        }
    }
}

/* ============================================================
 * APPLICATION ENTRY POINT
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "   ESP MULTI SENSOR TELEMETRY");
    ESP_LOGI(TAG, "====================================");

    /* Initialize communication interfaces. */
    lora_uart_init();
    can_init();

    ESP_LOGI(TAG,
             "RYLR993 target address=%d",
             PILOT_ADDRESS);

    ESP_LOGW(TAG,
             "Verify radio address, frequency "
             "and LoRa configuration.");

    /* Start telemetry processing task. */
    BaseType_t result = xTaskCreate(
        telemetry_task,
        "telemetry_task",
        6144,
        NULL,
        8,
        NULL);

    if (result != pdPASS) {

        ESP_LOGE(TAG,
                 "Failed to create telemetry task");

        abort();
    }

    ESP_LOGI(TAG, "Telemetry bridge started");
    ESP_LOGI(TAG, "Waiting for CAN telemetry...");
}
