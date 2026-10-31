
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
 * ESP MULTI SENSOR - TELEMETRY
 *
 * Board : ESP32-WROOM-32
 * SDK   : ESP-IDF v6.0.2
 *
 * Architecture:
 * Slaves -> Master -> Telemetry -> LoRa (RYLR993)
 *
 * Telemetry is a transparent CAN-to-LoRa bridge.
 * It preserves the original CAN IDs and payload bytes.
 * ============================================================ */


/* ================= TAG ================= */

#define TAG "TELEMETRY"


/* ================= CAN CONFIGURATION ================= */

#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4

#define CAN_BITRATE            500000

#define CAN_QUEUE_LEN          64
#define CAN_FRAME_SIZE         8


/* ================= SLAVE 1 CAN IDs ================= */

#define S1_ID_IMU              0x100
#define S1_ID_GYRO_HALL        0x101
#define S1_ID_TEMP_STATUS       0x102


/* ================= SLAVE 2 CAN IDs ================= */

#define S2_ID_ADC               0x200


/* ================= SLAVE 3 CAN IDs ================= */

#define S3_ID_TEMPERATURE       0x300
#define S3_ID_DISTANCE_PRESSURE 0x301
#define S3_ID_STATUS            0x302


/* ================= SLAVE 4 CAN IDs ================= */

#define S4_ID_RTDS              0x400
#define S4_ID_HALL              0x401
#define S4_ID_LIDAR             0x402


/* ================= LORA UART CONFIG ================= */

#define LORA_UART              UART_NUM_2

#define LORA_TX_GPIO           GPIO_NUM_17
#define LORA_RX_GPIO           GPIO_NUM_16

#define LORA_BAUD              9600

#define PILOT_ADDRESS          2


/* ================= PACKET CONFIG ================= */

#define BATCH_MAX              4
#define BATCH_WAIT_MS          50

#define RADIO_RESPONSE_TIMEOUT_MS 150

#define RADIO_PAYLOAD_MAX      240


/* ================= CAN MESSAGE STRUCTURE ================= */

typedef struct
{
    uint32_t id;
    uint8_t data[CAN_FRAME_SIZE];
    uint8_t length;

} can_msg_t;


/* ================= GLOBAL VARIABLES ================= */

static QueueHandle_t can_queue = NULL;

static twai_node_handle_t can_node = NULL;

static volatile uint32_t can_rx_count = 0;
static volatile uint32_t can_queue_drops = 0;

static uint32_t radio_seq = 0;

static uint32_t radio_send_ok = 0;
static uint32_t radio_send_fail = 0;


/* ============================================================
 * CAN ID VALIDATION
 *
 * Accepts only the CAN IDs defined by master/main.c
 * ============================================================ */

static bool is_telemetry_id(uint32_t id)
{
    switch (id)
    {
        /* Slave 1 */
        case S1_ID_IMU:
        case S1_ID_GYRO_HALL:
        case S1_ID_TEMP_STATUS:

        /* Slave 2 */
        case S2_ID_ADC:

        /* Slave 3 */
        case S3_ID_TEMPERATURE:
        case S3_ID_DISTANCE_PRESSURE:
        case S3_ID_STATUS:

        /* Slave 4 */
        case S4_ID_RTDS:
        case S4_ID_HALL:
        case S4_ID_LIDAR:

            return true;

        default:
            return false;
    }
}


/* ============================================================
 * SLAVE IDENTIFICATION
 *
 * Returns the originating Slave number based on CAN ID.
 * ============================================================ */

static uint8_t get_slave_number(uint32_t id)
{
    if (id >= 0x100 && id <= 0x102)
        return 1;

    if (id == 0x200)
        return 2;

    if (id >= 0x300 && id <= 0x302)
        return 3;

    if (id >= 0x400 && id <= 0x402)
        return 4;

    return 0;
}


/* ============================================================
 * CAN RECEIVE CALLBACK
 *
 * Runs in TWAI callback context.
 *
 * Responsibilities:
 * 1. Receive CAN frame
 * 2. Validate CAN ID
 * 3. Copy original frame to FreeRTOS queue
 *
 * No LoRa transmission or heavy processing here.
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

    /* Receive frame from TWAI */
    if (twai_node_receive_from_isr(
            handle,
            &frame) != ESP_OK)
    {
        return false;
    }

    /* Reject unknown CAN IDs */
    if (!is_telemetry_id(frame.header.id))
    {
        return false;
    }

    /* Current Master protocol uses 8-byte CAN frames */
    message.id = frame.header.id;
    message.length = CAN_FRAME_SIZE;

    BaseType_t higher_priority_task_woken = pdFALSE;

    /* Transfer frame to telemetry task */
    if (xQueueSendFromISR(
            can_queue,
            &message,
            &higher_priority_task_woken) != pdTRUE)
    {
        can_queue_drops++;
    }
    else
    {
        can_rx_count++;
    }

    return higher_priority_task_woken == pdTRUE;
}


/* ============================================================
 * CAN INITIALIZATION
 * ============================================================ */

static void can_init(void)
{
    /* Create queue */
    can_queue = xQueueCreate(
        CAN_QUEUE_LEN,
        sizeof(can_msg_t));

    if (can_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create CAN queue");
        abort();
    }

    /* Configure ESP32 on-chip TWAI */
    twai_onchip_node_config_t config = {

        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,

        .bit_timing.bitrate = CAN_BITRATE,

        .tx_queue_depth = 4,

        /* Telemetry receives only; does not transmit */
        .flags.enable_listen_only = 1,
    };

    ESP_ERROR_CHECK(
        twai_new_node_onchip(
            &config,
            &can_node));


    /* Register receive callback */
    twai_event_callbacks_t callbacks = {

        .on_rx_done = can_rx_callback,
    };

    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(
            can_node,
            &callbacks,
            NULL));


    /* Enable CAN controller */
    ESP_ERROR_CHECK(
        twai_node_enable(can_node));


    ESP_LOGI(TAG, "CAN initialized successfully");

    ESP_LOGI(TAG,
        "CAN bitrate : %d bit/s",
        CAN_BITRATE);

    ESP_LOGI(TAG,
        "CAN TX      : GPIO %d",
        CAN_TX_GPIO);

    ESP_LOGI(TAG,
        "CAN RX      : GPIO %d",
        CAN_RX_GPIO);

    ESP_LOGI(TAG,
        "Mode        : LISTEN ONLY");

    ESP_LOGI(TAG,
        "IDs         : 0x100-0x102, 0x200, "
        "0x300-0x302, 0x400-0x402");
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


    /* Install UART driver */
    ESP_ERROR_CHECK(
        uart_driver_install(
            LORA_UART,
            2048,
            2048,
            0,
            NULL,
            0));


    /* Configure UART parameters */
    ESP_ERROR_CHECK(
        uart_param_config(
            LORA_UART,
            &config));


    /* Configure UART GPIOs */
    ESP_ERROR_CHECK(
        uart_set_pin(
            LORA_UART,
            LORA_TX_GPIO,
            LORA_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE));


    ESP_LOGI(TAG,
        "LoRa UART initialized");

    ESP_LOGI(TAG,
        "UART        : UART2");

    ESP_LOGI(TAG,
        "Baudrate    : %d",
        LORA_BAUD);

    ESP_LOGI(TAG,
        "TX GPIO     : %d",
        LORA_TX_GPIO);

    ESP_LOGI(TAG,
        "RX GPIO     : %d",
        LORA_RX_GPIO);
}


/* ============================================================
 * CRC-16/CCITT-FALSE
 *
 * Polynomial : 0x1021
 * Initial   : 0xFFFF
 * XOR Out   : 0x0000
 * ============================================================ */

static uint16_t crc16_ccitt(
    const uint8_t *data,
    size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (uint16_t)(
                    (crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}


/* ============================================================
 * BYTE ARRAY TO HEX STRING
 * ============================================================ */

static void bytes_to_hex(
    char *out,
    const uint8_t *data,
    size_t length)
{
    static const char digits[] =
        "0123456789ABCDEF";

    for (size_t i = 0; i < length; i++)
    {
        out[2 * i] =
            digits[data[i] >> 4];

        out[2 * i + 1] =
            digits[data[i] & 0x0F];
    }

    out[2 * length] = '\0';
}


/* ============================================================
 * LORA BATCH TRANSMISSION
 *
 * Packet format:
 *
 * LM1,SEQ,COUNT,SLAVE,CAN_ID,LENGTH,DATA_HEX,...
 *
 * Example:
 *
 * LM1,1,2,1,100,8,0011223344550001,
 * 2,200,8,0001020304050607*CRC
 *
 * The CAN payload is forwarded without modification.
 * ============================================================ */

static bool lora_send_batch(
    const can_msg_t *messages,
    unsigned count)
{
    if (count == 0 || count > BATCH_MAX)
    {
        return false;
    }

    char payload[RADIO_PAYLOAD_MAX + 1];


    /* Start packet */
    int used = snprintf(
        payload,
        sizeof(payload),
        "LM1,%" PRIu32 ",%u",
        ++radio_seq,
        count);

    if (used < 0 ||
        used >= (int)sizeof(payload))
    {
        radio_send_fail++;
        return false;
    }


    /* Append every CAN frame */
    for (unsigned i = 0; i < count; i++)
    {
        char hex[17];

        bytes_to_hex(
            hex,
            messages[i].data,
            messages[i].length);


        uint8_t slave =
            get_slave_number(messages[i].id);


        int written = snprintf(
            payload + used,
            sizeof(payload) - (size_t)used,

            ",%u,%03" PRIX32 ",%u,%s",

            slave,
            messages[i].id,
            messages[i].length,
            hex);


        if (written < 0 ||
            written >=
            (int)(sizeof(payload) - (size_t)used))
        {
            radio_send_fail++;
            return false;
        }

        used += written;
    }


    /* Calculate CRC over packet contents */
    uint16_t crc = crc16_ccitt(
        (const uint8_t *)payload,
        (size_t)used);


    /* Append CRC */
    int written = snprintf(
        payload + used,
        sizeof(payload) - (size_t)used,
        "*%04X",
        crc);


    if (written < 0 ||
        written >=
        (int)(sizeof(payload) - (size_t)used))
    {
        radio_send_fail++;
        return false;
    }

    used += written;


    /* Build RYLR993 AT command */
    char command[RADIO_PAYLOAD_MAX + 40];

    int command_len = snprintf(
        command,
        sizeof(command),

        "AT+SEND=%d,%d,%s\r\n",

        PILOT_ADDRESS,
        used,
        payload);


    if (command_len <= 0 ||
        command_len >= (int)sizeof(command))
    {
        radio_send_fail++;
        return false;
    }


    /* Clear old UART input */
    uart_flush_input(LORA_UART);


    /* Send packet */
    int sent = uart_write_bytes(
        LORA_UART,
        command,
        command_len);


    if (sent != command_len)
    {
        radio_send_fail++;

        ESP_LOGE(TAG,
            "LoRa UART write failed");

        return false;
    }


    /* Wait for module response */
    char response[160] = {0};

    int got = uart_read_bytes(
        LORA_UART,
        (uint8_t *)response,
        sizeof(response) - 1,

        pdMS_TO_TICKS(
            RADIO_RESPONSE_TIMEOUT_MS));


    bool ok = (
        got > 0 &&
        (strstr(response, "+OK") != NULL ||
         strstr(response, "OK") != NULL));


    if (ok)
    {
        radio_send_ok++;
    }
    else
    {
        radio_send_fail++;
    }


    ESP_LOGI(TAG,
        "LoRa seq=%" PRIu32
        " frames=%u response=%s",

        radio_seq,
        count,

        ok ? "OK" : "NO RESPONSE");


    return ok;
}


/* ============================================================
 * TELEMETRY PROCESSING TASK
 *
 * Receives queued CAN frames and batches them for LoRa.
 * ============================================================ */

static void telemetry_task(void *arg)
{
    (void)arg;

    can_msg_t batch[BATCH_MAX];


    while (1)
    {
        unsigned count = 0;


        /* Wait for first CAN frame */
        if (xQueueReceive(
                can_queue,
                &batch[count],
                pdMS_TO_TICKS(BATCH_WAIT_MS)) == pdTRUE)
        {
            count = 1;


            /* Collect additional frames */
            while (count < BATCH_MAX &&
                   xQueueReceive(
                       can_queue,
                       &batch[count],
                       0) == pdTRUE)
            {
                count++;
            }


            /* Forward batch through LoRa */
            lora_send_batch(
                batch,
                count);
        }
        else
        {
            vTaskDelay(
                pdMS_TO_TICKS(10));
        }


        /* Periodic statistics */
        ESP_LOGI(TAG,
            "CAN RX=%" PRIu32
            " Queue drops=%" PRIu32
            " LoRa OK=%" PRIu32
            " LoRa FAIL=%" PRIu32,

            can_rx_count,
            can_queue_drops,
            radio_send_ok,
            radio_send_fail);
    }
}


/* ============================================================
 * APPLICATION ENTRY POINT
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG,
        "========================================");

    ESP_LOGI(TAG,
        "    ESP MULTI SENSOR TELEMETRY");

    ESP_LOGI(TAG,
        "========================================");


    ESP_LOGI(TAG,
        "Initializing LoRa UART...");

    lora_uart_init();


    ESP_LOGI(TAG,
        "Initializing CAN/TWAI...");

    can_init();


    ESP_LOGI(TAG,
        "Telemetry bridge initialized");


    ESP_LOGW(TAG,
        "Master CAN bitrate must be 500 kbps");

    ESP_LOGW(TAG,
        "Ensure proper CAN transceiver and termination");

    ESP_LOGW(TAG,
        "RYLR993 address=%d, band=865 MHz",
        PILOT_ADDRESS);


    /* Create telemetry task */
    BaseType_t result = xTaskCreate(
        telemetry_task,

        "telemetry_task",

        6144,

        NULL,

        8,

        NULL);


    if (result != pdPASS)
    {
        ESP_LOGE(TAG,
            "Failed to create telemetry task");

        abort();
    }


    ESP_LOGI(TAG,
        "Waiting for CAN data from Master...");
}
