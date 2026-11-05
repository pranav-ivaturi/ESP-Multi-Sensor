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

/* Telemetry bridge plus a narrow, validated relay-command return path. */
#define TAG "LOOPMIT_TELEMETRY"
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_BITRATE 500000
#define CAN_QUEUE_LEN 64
#define CAN_DLEN 8
#define TLM_ID_MIN 0x500
#define TLM_ID_MAX 0x50A
#define RELAY_COMMAND_CAN_ID 0x600
#define RELAY_COMMAND_MAGIC 0xA5
#define RELAY_COMMAND_TRAILER 0x5A
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
static uint16_t last_relay_sequence;
static bool have_relay_sequence;

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

static uint8_t relay_command_checksum(const uint8_t data[8])
{
    uint8_t checksum = 0;
    for (int i = 0; i < 6; ++i) checksum ^= data[i];
    return checksum;
}

static bool parse_u32(const char *text, int base, uint32_t *value)
{
    if (text == NULL || *text == '\0') return false;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, base);
    if (end == text || *end != '\0' || parsed > UINT32_MAX) return false;
    *value = (uint32_t)parsed;
    return true;
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
        .tx_queue_depth = 4,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &can_node));
    twai_event_callbacks_t cb = {.on_rx_done = can_rx_callback};
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(can_node, &cb, NULL));
    ESP_ERROR_CHECK(twai_node_enable(can_node));
    ESP_LOGI(TAG, "CAN bridge: %d bps, TX=%d RX=%d, Master IDs 0x500-0x50A",
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

/* GUI/Pilot command: LC1,<sequence>,<0|1>*<CRC16>. Only LoRa source 2 is accepted. */
static void forward_relay_command(uint16_t sequence, uint8_t requested_state)
{
    uint8_t data[8] = {RELAY_COMMAND_MAGIC, requested_state, 0, 0, 'G', 'U', 0,
                       RELAY_COMMAND_TRAILER};
    data[2] = (uint8_t)(sequence >> 8);
    data[3] = (uint8_t)sequence;
    data[6] = relay_command_checksum(data);
    twai_frame_t frame = {.header.id = RELAY_COMMAND_CAN_ID, .header.ide = false,
                          .buffer = data, .buffer_len = sizeof(data)};
    if (twai_node_transmit(can_node, &frame, 20) != ESP_OK)
        ESP_LOGW(TAG, "Relay command could not be forwarded to Master");
}

static void process_relay_payload(char *payload)
{
    char *star = strrchr(payload, '*');
    uint32_t given = 0, sequence = 0, state = 0;
    if (!star || strlen(star + 1) != 4 || !parse_u32(star + 1, 16, &given) ||
        crc16_ccitt_false((const uint8_t *)payload, (size_t)(star - payload)) != (uint16_t)given)
        return;
    *star = '\0';
    char *save = NULL;
    char *version = strtok_r(payload, ",", &save);
    if (!version || strcmp(version, "LC1") != 0 ||
        !parse_u32(strtok_r(NULL, ",", &save), 10, &sequence) || sequence > UINT16_MAX ||
        !parse_u32(strtok_r(NULL, ",", &save), 10, &state) || state > 1 ||
        strtok_r(NULL, ",", &save) != NULL)
        return;
    uint16_t seq16 = (uint16_t)sequence;
    uint16_t delta = (uint16_t)(seq16 - last_relay_sequence);
    if (have_relay_sequence && (delta == 0 || delta >= 0x8000))
        return;
    have_relay_sequence = true;
    last_relay_sequence = seq16;
    forward_relay_command(seq16, (uint8_t)state);
}

/* RYLR receive form: +RCV=<source>,<length>,<payload>,<rssi>,<snr>. */
static void lora_command_receive_task(void *arg)
{
    (void)arg;
    char line[PAYLOAD_MAX + 80];
    size_t used = 0;
    while (1) {
        uint8_t ch;
        if (uart_read_bytes(LORA_UART, &ch, 1, pdMS_TO_TICKS(100)) <= 0)
            continue;
        if (ch == '\r') continue;
        if (ch != '\n' && used < sizeof(line) - 1) { line[used++] = (char)ch; continue; }
        line[used] = '\0'; used = 0;
        if (strncmp(line, "+RCV=", 5) != 0) continue;
        char *source = line + 5;
        char *comma1 = strchr(source, ',');
        if (!comma1) continue;
        *comma1 = '\0';
        char *length = comma1 + 1;
        char *comma2 = strchr(length, ',');
        if (!comma2) continue;
        *comma2 = '\0';
        char *payload = comma2 + 1;
        uint32_t src = 0, declared = 0;
        if (!parse_u32(source, 10, &src) || !parse_u32(length, 10, &declared) ||
            src != PILOT_ADDRESS || declared == 0 || declared >= sizeof(line)) continue;
        if (payload[declared] != ',') continue;
        payload[declared] = '\0';
        process_relay_payload(payload);
    }
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

    int sent = uart_write_bytes(LORA_UART, cmd, (size_t)clen);
    if (sent != clen) {
        radio_fail++;
        ESP_LOGE(TAG, "UART write failed %d/%d", sent, clen);
        return false;
    }
    (void)uart_wait_tx_done(LORA_UART, pdMS_TO_TICKS(150));

    /* UART RX is owned by the command task so incoming GUI commands are never lost. */
    radio_ok++;
    ESP_LOGI(TAG, "LoRa seq=%" PRIu32 " frames=%u queued", radio_seq, count);
    return true;
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
    if (xTaskCreate(lora_command_receive_task, "lora_command_rx", 4096, NULL, 9, NULL) != pdPASS)
        abort();
    ESP_LOGW(TAG, "Pod RYLR993 address=1; Pilot address=2; band=865 MHz; match radio parameters");
    if (xTaskCreate(telemetry_task, "telemetry_task", 6144, NULL, 8, NULL) != pdPASS)
        abort();
    ESP_LOGI(TAG, "Ready: telemetry 0x500-0x50A and validated relay commands from LoRa source %d", PILOT_ADDRESS);
}
