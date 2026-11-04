#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define LORA_UART UART_NUM_2
#define LORA_TX_GPIO GPIO_NUM_17
#define LORA_RX_GPIO GPIO_NUM_16
#define LORA_BAUD 9600
#define LORA_LINE_MAX 512
#define MAX_FRAMES 4
#define POD_ADDRESS 1
#define TLM_ID_MIN 0x500
#define TLM_ID_MAX 0x509

static uint32_t packets_received, packets_lost, bad_crc, malformed, duplicate_old;
static uint32_t last_seq;
static bool have_seq;
static int64_t rssi_sum, snr_sum;
static uint32_t radio_samples;

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

static bool parse_u32(const char *s, int base, uint32_t *v)
{
    if (!s || !*s) return false;
    char *e = NULL;
    unsigned long x = strtoul(s, &e, base);
    if (e == s || *e || x > UINT32_MAX) return false;
    *v = (uint32_t)x;
    return true;
}

static bool parse_hex(const char *s, uint8_t *out, size_t n)
{
    if (!s || strlen(s) != 2 * n) return false;
    for (size_t i = 0; i < n; ++i) {
        char b[3] = {s[2 * i], s[2 * i + 1], 0};
        char *e = NULL;
        unsigned long x = strtoul(b, &e, 16);
        if (e == b || *e || x > 255) return false;
        out[i] = (uint8_t)x;
    }
    return true;
}

static uint16_t be16u(const uint8_t *d)
{
    return (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
}
static int16_t be16s(const uint8_t *d) { return (int16_t)be16u(d); }
static uint32_t be32u(const uint8_t *d)
{
    return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
           ((uint32_t)d[2] << 8) | (uint32_t)d[3];
}

static void uart_init(void)
{
    const uart_config_t c = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART, 4096, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART, &c));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART, LORA_TX_GPIO, LORA_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void emit_raw_can(uint32_t id, uint8_t len, const uint8_t data[8])
{
    printf("{\"type\":\"can_frame\",\"id\":\"0x%03" PRIX32 "\",\"length\":%u,\"data\":\"",
           id, len);
    for (uint8_t i = 0; i < len; ++i) printf("%02X", data[i]);
    printf("\"}\n");
}

/* Decode the Master's 0x500-0x509 telemetry map. Multi-byte values are BIG-ENDIAN. */
static void emit_sensor_json(uint32_t id, const uint8_t d[8])
{
    switch (id) {
    case 0x500: /* Original 0x100: IMU AX, AY, AZ, sequence; raw scale retained by Master protocol. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x500\",\"sensor\":\"imu\",\"ax_raw\":%d,\"ay_raw\":%d,\"az_raw\":%d,\"sequence\":%u}\n",
               be16s(&d[0]), be16s(&d[2]), be16s(&d[4]), be16u(&d[6]));
        break;
    case 0x501: /* Original 0x101: gyro GX/GY/GZ and Hall ADC. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x501\",\"sensor\":\"gyro_hall\",\"gx_raw\":%d,\"gy_raw\":%d,\"gz_raw\":%d,\"hall_raw\":%u}\n",
               be16s(&d[0]), be16s(&d[2]), be16s(&d[4]), be16u(&d[6]));
        break;
    case 0x502: /* Original 0x102: temperatures x100, Hall D0, RTD faults, status. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x502\",\"sensor\":\"slave1_temp_status\",\"temperature1_c\":%.2f,\"temperature2_c\":%.2f,\"hall_d0\":%u,\"rtd1_fault\":%u,\"rtd2_fault\":%u,\"status\":%u}\n",
               be16s(&d[0]) / 100.0, be16s(&d[2]) / 100.0, d[4], d[5], d[6], d[7]);
        break;
    case 0x503: /* Original 0x200: raw ADC, ADC mV, input mV, sequence. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x503\",\"sensor\":\"dc_link_voltage\",\"raw_adc\":%d,\"adc_voltage_v\":%.3f,\"input_voltage_v\":%.3f,\"sequence\":%u}\n",
               be16s(&d[0]), be16u(&d[2]) / 1000.0, be16u(&d[4]) / 1000.0, be16u(&d[6]));
        break;
    case 0x504: /* Original 0x300: four temperature values x100. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x504\",\"sensor\":\"slave3_temperatures\",\"t1_ambient_c\":%.2f,\"t1_object_c\":%.2f,\"t2_ambient_c\":%.2f,\"t2_object_c\":%.2f}\n",
               be16s(&d[0]) / 100.0, be16s(&d[2]) / 100.0,
               be16s(&d[4]) / 100.0, be16s(&d[6]) / 100.0);
        break;
    case 0x505: { /* Original 0x301: LiDAR mm, pressure Pa, sequence. */
        uint16_t lidar_mm = be16u(&d[0]);
        uint32_t pressure_pa = be32u(&d[2]);
        printf("{\"type\":\"sensor\",\"can_id\":\"0x505\",\"sensor\":\"distance_pressure\",\"lidar_mm\":%u,\"pressure_pa\":%" PRIu32 ",\"pressure_bar\":%.3f,\"sequence\":%u}\n",
               lidar_mm, pressure_pa, pressure_pa / 100000.0, be16u(&d[6]));
        break;
    }
    case 0x506: { /* Original 0x302: status bits. */
        uint8_t s = d[0];
        printf("{\"type\":\"sensor\",\"can_id\":\"0x506\",\"sensor\":\"slave3_status\",\"status\":%u,\"temp1_ok\":%s,\"temp2_ok\":%s,\"lidar_ok\":%s,\"pressure_ok\":%s}\n",
               s, (s & 0x01) ? "true" : "false", (s & 0x02) ? "true" : "false",
               (s & 0x04) ? "true" : "false", (s & 0x08) ? "true" : "false");
        break;
    }
    case 0x507: /* Original 0x400: RTD temps x100, status, sequence. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x507\",\"sensor\":\"slave4_rtd\",\"temperature1_c\":%.2f,\"temperature2_c\":%.2f,\"status\":%u,\"sequence\":%u}\n",
               be16s(&d[0]) / 100.0, be16s(&d[2]) / 100.0, d[4], be16u(&d[5]));
        break;
    case 0x508: /* Original 0x401: Hall raw, mV, status, sequence. */
        printf("{\"type\":\"sensor\",\"can_id\":\"0x508\",\"sensor\":\"slave4_hall\",\"hall_raw\":%u,\"hall_voltage_v\":%.3f,\"status\":%u,\"sequence\":%u}\n",
               be16u(&d[0]), be16u(&d[2]) / 1000.0, d[4], be16u(&d[5]));
        break;
    case 0x509: { /* Original 0x201: Slave 2 signed LiDAR distance, sequence, status. */
        int16_t distance_mm = be16s(&d[0]);
        if (distance_mm < 0)
            printf("{\"type\":\"sensor\",\"can_id\":\"0x509\",\"sensor\":\"slave2_lidar\",\"available\":false,\"status\":%u,\"sequence\":%u}\n",
                   d[4], be16u(&d[2]));
        else
            printf("{\"type\":\"sensor\",\"can_id\":\"0x509\",\"sensor\":\"slave2_lidar\",\"available\":true,\"distance_mm\":%d,\"status\":%u,\"sequence\":%u}\n",
                   distance_mm, d[4], be16u(&d[2]));
        break;
    }
    default:
        printf("{\"type\":\"sensor\",\"can_id\":\"0x%03" PRIX32 "\",\"sensor\":\"unknown\"}\n", id);
        break;
    }
}

static void emit_stats(void)
{
    uint32_t total = packets_received + packets_lost;
    double rp = total ? 100.0 * (double)packets_received / total : 0.0;
    double lp = total ? 100.0 * (double)packets_lost / total : 0.0;
    double ar = radio_samples ? (double)rssi_sum / radio_samples : 0.0;
    double as = radio_samples ? (double)snr_sum / radio_samples : 0.0;
    printf("{\"type\":\"link_stats\",\"packets_received\":%" PRIu32
           ",\"packets_lost\":%" PRIu32 ",\"received_pct\":%.2f,\"lost_pct\":%.2f"
           ",\"avg_rssi\":%.2f,\"avg_snr\":%.2f,\"bad_crc\":%" PRIu32
           ",\"malformed_packets\":%" PRIu32 ",\"duplicate_or_old\":%" PRIu32 "}\n",
           packets_received, packets_lost, rp, lp, ar, as, bad_crc, malformed, duplicate_old);
}

static void process_payload(char *p, int rssi, int snr)
{
    char *star = strrchr(p, '*');
    if (!star || strlen(star + 1) != 4) { bad_crc++; return; }
    uint32_t given;
    if (!parse_u32(star + 1, 16, &given)) { bad_crc++; return; }
    uint16_t calc = crc16_ccitt_false((const uint8_t *)p, (size_t)(star - p));
    if (calc != (uint16_t)given) { bad_crc++; return; }
    *star = '\0';

    if (strncmp(p, "LM1,", 4) != 0) { malformed++; return; }
    char *save = NULL;
    (void)strtok_r(p, ",", &save);
    uint32_t seq, count;
    if (!parse_u32(strtok_r(NULL, ",", &save), 10, &seq) ||
        !parse_u32(strtok_r(NULL, ",", &save), 10, &count) ||
        count == 0 || count > MAX_FRAMES) { malformed++; return; }

    uint32_t ids[MAX_FRAMES];
    uint8_t lens[MAX_FRAMES], data[MAX_FRAMES][8];
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t x;
        if (!parse_u32(strtok_r(NULL, ",", &save), 16, &x) ||
            x < TLM_ID_MIN || x > TLM_ID_MAX) { malformed++; return; }
        ids[i] = x;
        if (!parse_u32(strtok_r(NULL, ",", &save), 10, &x) || x != 8) { malformed++; return; }
        lens[i] = (uint8_t)x;
        if (!parse_hex(strtok_r(NULL, ",", &save), data[i], lens[i])) { malformed++; return; }
    }
    if (strtok_r(NULL, ",", &save) != NULL) { malformed++; return; }

    if (have_seq) {
        uint32_t delta = seq - last_seq;
        if (delta == 0 || delta >= 0x80000000u) { duplicate_old++; return; }
        if (delta > 1) packets_lost += delta - 1;
    }
    have_seq = true;
    last_seq = seq;
    packets_received++;
    rssi_sum += rssi;
    snr_sum += snr;
    radio_samples++;

    for (uint32_t i = 0; i < count; ++i) {
        emit_raw_can(ids[i], lens[i], data[i]);
        emit_sensor_json(ids[i], data[i]);
    }
    emit_stats();
}

static void process_module_line(char *line)
{
    if (strncmp(line, "+RCV=", 5) != 0) return;
    char *src = line + 5;
    char *c1 = strchr(src, ',');
    if (!c1) { malformed++; return; }
    *c1 = '\0';
    uint32_t source;
    if (!parse_u32(src, 10, &source) || source != POD_ADDRESS) return;

    char *lenstr = c1 + 1;
    char *c2 = strchr(lenstr, ',');
    if (!c2) { malformed++; return; }
    *c2 = '\0';
    uint32_t declared;
    if (!parse_u32(lenstr, 10, &declared)) { malformed++; return; }

    char *payload = c2 + 1;
    char *csnr = strrchr(payload, ',');
    if (!csnr) { malformed++; return; }
    *csnr = '\0';
    char *snrstr = csnr + 1;
    char *crssi = strrchr(payload, ',');
    if (!crssi) { malformed++; return; }
    *crssi = '\0';
    char *rssistr = crssi + 1;

    char *e = NULL;
    long rssi = strtol(rssistr, &e, 10);
    if (e == rssistr || *e || rssi < -200 || rssi > 100) { malformed++; return; }
    long snr = strtol(snrstr, &e, 10);
    if (e == snrstr || *e || snr < -100 || snr > 100) { malformed++; return; }
    if (declared != strlen(payload)) { malformed++; return; }
    process_payload(payload, (int)rssi, (int)snr);
}

static void receive_task(void *arg)
{
    (void)arg;
    char line[LORA_LINE_MAX];
    size_t used = 0;
    uint8_t ch;
    while (1) {
        int got = uart_read_bytes(LORA_UART, &ch, 1, pdMS_TO_TICKS(100));
        if (got <= 0) continue;
        if (ch == '\r') continue;
        if (ch == '\n') {
            line[used] = '\0';
            if (used) process_module_line(line);
            used = 0;
        } else if (used < sizeof(line) - 1) {
            line[used++] = (char)ch;
        } else {
            malformed++;
            used = 0;
        }
    }
}

void app_main(void)
{
    uart_init();
    printf("{\"type\":\"boot\",\"device\":\"LoopMIT Pilot telemetry receiver\"}\n");
    printf("{\"type\":\"info\",\"message\":\"JSON on USB serial; Pilot RYLR993 address=2; expected pod source=1\"}\n");
    if (xTaskCreate(receive_task, "lora_rx", 6144, NULL, 8, NULL) != pdPASS)
        abort();
}
