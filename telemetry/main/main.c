#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* Transparent bridge: Master IDs 0x500-0x509 -> RYLR993 -> Pilot. */
#define TAG "LOOPMIT_TELEMETRY"
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_BITRATE 500000
#define CAN_QUEUE_LEN 64
#define CAN_DLEN 8
#define TLM_ID_MIN 0x500
#define TLM_ID_MAX 0x509
#define LORA_UART UART_NUM_2
#define LORA_TX_GPIO GPIO_NUM_17
#define LORA_RX_GPIO GPIO_NUM_16
#define LORA_BAUD 9600
#define PILOT_ADDRESS 2
#define BATCH_MAX 4
#define BATCH_WAIT_MS 50
#define RADIO_TIMEOUT_MS 300
#define PAYLOAD_MAX 240

typedef struct {
    uint32_t id;
    uint8_t data[CAN_DLEN];
    uint8_t len;
} can_msg_t;

static QueueHandle_t can_queue;
static twai_node_handle_t can_node;
static volatile uint32_t can_rx_count, can_queue_drops;
static uint32_t radio_seq, radio_ok, radio_fail;

static bool is_master_telemetry_id(uint32_t id)
{
    return id >= TLM_ID_MIN && id <= TLM_ID_MAX;
}

static uint16_t crc16_ccitt_false(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void bytes_to_hex(char *out, const uint8_t *data, size_t n)
{
    static const char h[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; ++i) {
        out[2 * i] = h[data[i] >> 4];
        out[2 * i + 1] = h[data[i] & 0x0F];
    }
    out[2 * n] = '\0';
}

static bool can_rx_callback(twai_node_handle_t handle,
                            const twai_rx_done_event_data_t *edata,
                            void *ctx)
{
    (void)edata;
    (void)ctx;
    can_msg_t m = {0};
    twai_frame_t f = {.buffer = m.data, .buffer_len = sizeof(m.data)};
    if (twai_node_receive_from_isr(handle, &f) != ESP_OK)
        return false;

    /* Reject extended frames and all original Slave IDs. */
    if (f.header.ide || !is_master_telemetry_id(f.header.id))
        return false;

    /* Master protocol defines all published telemetry frames as 8-byte Classical CAN. */
    m.id = f.header.id;
    m.len = CAN_DLEN;

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(can_queue, &m, &woken) != pdTRUE)
        can_queue_drops++;
    else
        can_rx_count++;
    return woken == pdTRUE;
}

static void can_init(void)
{
    can_queue = xQueueCreate(CAN_QUEUE_LEN, sizeof(can_msg_t));
    if (!can_queue) {
        ESP_LOGE(TAG, "CAN queue creation failed");
        abort();
    }

    twai_onchip_node_config_t cfg = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 1,
        .flags.enable_listen_only = 1,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &can_node));
    twai_event_callbacks_t cb = {.on_rx_done = can_rx_callback};
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(can_node, &cb, NULL));
    ESP_ERROR_CHECK(twai_node_enable(can_node));
    ESP_LOGI(TAG, "CAN listen-only: %d bps, TX=%d RX=%d, Master IDs 0x500-0x509",
             CAN_BITRATE, CAN_TX_GPIO, CAN_RX_GPIO);
}

static void lora_uart_init(void)
{
    const uart_config_t c = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART, &c));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART, LORA_TX_GPIO, LORA_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/* Read a complete CR/LF-terminated response line; do not assume one UART read is complete. */
static int read_response(char *out, size_t cap, TickType_t timeout)
{
    if (cap < 2) return -1;
    size_t n = 0;
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t ch;
        int got = uart_read_bytes(LORA_UART, &ch, 1, pdMS_TO_TICKS(10));
        if (got <= 0) continue;
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (!n) continue;
            out[n] = '\0';
            return (int)n;
        }
        if (n < cap - 1) out[n++] = (char)ch;
        else {
            out[n] = '\0';
            return -1;
        }
    }
    out[n] = '\0';
    return n ? (int)n : 0;
}

/* LM1,SEQ,COUNT,CAN_ID,LENGTH,DATA_HEX[,CAN_ID,LENGTH,DATA_HEX...]*CRC */
static bool send_batch(const can_msg_t *msgs, unsigned count)
{
    if (!count || count > BATCH_MAX) return false;

    char payload[PAYLOAD_MAX + 1];
    int used = snprintf(payload, sizeof(payload), "LM1,%" PRIu32 ",%u", ++radio_seq, count);
    if (used < 0 || used >= (int)sizeof(payload)) {
        radio_fail++;
        return false;
    }

    for (unsigned i = 0; i < count; ++i) {
        if (!is_master_telemetry_id(msgs[i].id) || msgs[i].len != CAN_DLEN) {
            radio_fail++;
            ESP_LOGE(TAG, "Rejected unexpected CAN frame ID=0x%03" PRIX32 " len=%u",
                     msgs[i].id, msgs[i].len);
            return false;
        }
        char hex[CAN_DLEN * 2 + 1];
        bytes_to_hex(hex, msgs[i].data, msgs[i].len);
        int w = snprintf(payload + used, sizeof(payload) - (size_t)used,
                         ",%03" PRIX32 ",%u,%s", msgs[i].id, msgs[i].len, hex);
        if (w < 0 || w >= (int)(sizeof(payload) - (size_t)used)) {
            radio_fail++;
            return false;
        }
        used += w;
    }

    uint16_t crc = crc16_ccitt_false((const uint8_t *)payload, (size_t)used);
    int w = snprintf(payload + used, sizeof(payload) - (size_t)used, "*%04X", crc);
    if (w < 0 || w >= (int)(sizeof(payload) - (size_t)used)) {
        radio_fail++;
        return false;
    }
    used += w;

    char cmd[PAYLOAD_MAX + 40];
    int clen = snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s\r\n", PILOT_ADDRESS, used, payload);
    if (clen <= 0 || clen >= (int)sizeof(cmd)) {
        radio_fail++;
        return false;
    }

    /* The module is used only for TX on this node; discard stale response bytes before command. */
    uart_flush_input(LORA_UART);
    int sent = uart_write_bytes(LORA_UART, cmd, (size_t)clen);
    if (sent != clen) {
        radio_fail++;
        ESP_LOGE(TAG, "UART write failed %d/%d", sent, clen);
        return false;
    }
    (void)uart_wait_tx_done(LORA_UART, pdMS_TO_TICKS(150));

    char response[128];
    int nr = read_response(response, sizeof(response), pdMS_TO_TICKS(RADIO_TIMEOUT_MS));
    bool ok = nr > 0 && (strstr(response, "+OK") != NULL || strcmp(response, "OK") == 0);
    if (ok) radio_ok++;
    else {
        radio_fail++;
        ESP_LOGW(TAG, "RYLR response: %s", nr > 0 ? response : (nr < 0 ? "line too long" : "timeout"));
    }
    ESP_LOGI(TAG, "LoRa seq=%" PRIu32 " frames=%u accepted=%d", radio_seq, count, ok);
    return ok;
}

static void telemetry_task(void *arg)
{
    (void)arg;
    can_msg_t batch[BATCH_MAX];
    while (1) {
        unsigned n = 0;
        if (xQueueReceive(can_queue, &batch[0], pdMS_TO_TICKS(BATCH_WAIT_MS)) == pdTRUE) {
            n = 1;
            while (n < BATCH_MAX && xQueueReceive(can_queue, &batch[n], 0) == pdTRUE)
                n++;
            (void)send_batch(batch, n);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "Master frames=%" PRIu32 " queue drops=%" PRIu32
                      " LoRa OK=%" PRIu32 " fail=%" PRIu32,
                 can_rx_count, can_queue_drops, radio_ok, radio_fail);
    }
}

void app_main(void)
{
    lora_uart_init();
    can_init();
    ESP_LOGW(TAG, "Pod RYLR993 address=1; Pilot address=2; band=865 MHz; match radio parameters");
    if (xTaskCreate(telemetry_task, "telemetry_task", 6144, NULL, 8, NULL) != pdPASS)
        abort();
    ESP_LOGI(TAG, "Ready: forwarding only Master telemetry IDs 0x500-0x509");
}