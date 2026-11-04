/*
 * ============================================================
 *              ESP MULTI SENSOR - MASTER
 * ============================================================
 *
 *  Board : ESP32-WROOM-32 DevKit
 *  SDK   : ESP-IDF v6.x
 *
 *  PURPOSE
 *    Central CAN receiver for the ESP Multi Sensor system.
 *
 *  CAN
 *    Bitrate : 500 kbit/s
 *    TX      : GPIO 5
 *    RX      : GPIO 4
 *    Format  : Standard 11-bit CAN
 *
 *  IMPORTANT
 *    All multi-byte sensor values use BIG-ENDIAN encoding.
 *
 *  CURRENT VERSION
 *    - Initializes CAN/TWAI
 *    - Receives CAN frames
 *    - Decodes all four slave nodes
 *    - Prints decoded sensor data
 *    - Tracks slave communication
 *
 *  NO HARDWARE CONTROL IS IMPLEMENTED YET.
 *
 * ============================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"


/* ============================================================
 * TAG
 * ============================================================ */

static const char *TAG = "MASTER";


/* ============================================================
 * CAN CONFIGURATION
 * ============================================================ */

#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4

#define CAN_BITRATE             500000

#define CAN_RX_QUEUE_LENGTH     32

#define CAN_FRAME_LENGTH        8


/* ============================================================
 * SLAVE 1 CAN IDs
 * ============================================================ */

#define S1_ID_IMU               0x100
#define S1_ID_GYRO_HALL         0x101
#define S1_ID_TEMP_STATUS       0x102


/* ============================================================
 * SLAVE 2 CAN ID
 * ============================================================ */

#define S2_ID_ADC               0x200
#define S2_ID_LIDAR             0x201


/* ============================================================
 * SLAVE 3 CAN IDs
 * ============================================================ */

#define S3_ID_TEMPERATURE       0x300
#define S3_ID_DISTANCE_PRESSURE 0x301
#define S3_ID_STATUS             0x302


/* ============================================================
 * SLAVE 4 CAN IDs
 * ============================================================ */

#define S4_ID_RTDS              0x400
#define S4_ID_HALL              0x401


/* ============================================================
 * CAN HANDLE
 * ============================================================ */

static twai_node_handle_t can_node = NULL;


/* ============================================================
 * RECEIVED FRAME STRUCTURE
 *
 * The ISR callback copies received CAN data into this structure
 * and places it into the FreeRTOS queue.
 * ============================================================ */

typedef struct
{
    uint32_t id;
    uint8_t data[CAN_FRAME_LENGTH];
    uint8_t length;

} can_rx_message_t;


static QueueHandle_t can_rx_queue = NULL;


/* ============================================================
 * SLAVE ONLINE TRACKING
 *
 * Used later for Master health monitoring.
 * ============================================================ */

static volatile uint32_t slave1_last_rx = 0;
static volatile uint32_t slave2_last_rx = 0;
static volatile uint32_t slave3_last_rx = 0;
static volatile uint32_t slave4_last_rx = 0;

static uint32_t master_tlm_tx_ok = 0;
static uint32_t master_tlm_tx_fail = 0;


/* ============================================================
 * BIG-ENDIAN HELPERS
 * ============================================================ */

static uint16_t read_be16(
    const uint8_t *data
)
{
    return
        ((uint16_t)data[0] << 8) |
        ((uint16_t)data[1]);
}


static int16_t read_be_signed16(
    const uint8_t *data
)
{
    return (int16_t)read_be16(data);
}


static uint32_t read_be32(
    const uint8_t *data
)
{
    return
        ((uint32_t)data[0] << 24) |
        ((uint32_t)data[1] << 16) |
        ((uint32_t)data[2] << 8)  |
        ((uint32_t)data[3]);
}



/* ============================================================
 * MASTER TELEMETRY PUBLISHER
 *
 * Original Slave frames are decoded by the functions above.
 * After decoding, the same 8 data bytes are republished using
 * Master-owned IDs 0x500..0x509. Telemetry ESP forwards these
 * frames unchanged; Pilot decodes them using this mapping.
 * ============================================================ */
static uint32_t master_telemetry_id(uint32_t slave_id)
{
    switch (slave_id)
    {
        case S1_ID_IMU:               return 0x500;
        case S1_ID_GYRO_HALL:         return 0x501;
        case S1_ID_TEMP_STATUS:       return 0x502;
        case S2_ID_ADC:               return 0x503;
        case S2_ID_LIDAR:             return 0x509;
        case S3_ID_TEMPERATURE:       return 0x504;
        case S3_ID_DISTANCE_PRESSURE: return 0x505;
        case S3_ID_STATUS:            return 0x506;
        case S4_ID_RTDS:              return 0x507;
        case S4_ID_HALL:              return 0x508;
        default:                      return 0;
    }
}

static void publish_master_telemetry(const can_rx_message_t *message)
{
    uint32_t out_id = master_telemetry_id(message->id);
    if (out_id == 0 || message->length != CAN_FRAME_LENGTH)
        return;

    /*
     * TWAI transmit is asynchronous. We wait for completion before
     * returning so the stack-backed frame/data remain valid.
     */
    twai_frame_t tx_frame = {
        .header.id = out_id,
        .header.ide = false,  /* Standard 11-bit CAN identifier */
        .buffer = (uint8_t *)message->data,
        .buffer_len = CAN_FRAME_LENGTH,
    };

    esp_err_t err = twai_node_transmit(can_node, &tx_frame, 20);
    if (err != ESP_OK)
    {
        master_tlm_tx_fail++;
        ESP_LOGE(TAG, "Telemetry CAN enqueue failed ID=0x%03" PRIX32 ": %s",
                 out_id, esp_err_to_name(err));
        return;
    }

    err = twai_node_transmit_wait_all_done(can_node, 100);
    if (err != ESP_OK)
    {
        master_tlm_tx_fail++;
        ESP_LOGE(TAG, "Telemetry CAN TX not confirmed ID=0x%03" PRIX32 ": %s",
                 out_id, esp_err_to_name(err));
        return;
    }

    master_tlm_tx_ok++;
    ESP_LOGD(TAG, "Published Slave ID 0x%03" PRIX32 " as Master ID 0x%03" PRIX32,
             message->id, out_id);
}

/* ============================================================
 * CAN RECEIVE CALLBACK
 *
 * IMPORTANT:
 * This callback executes in the TWAI receive callback context.
 *
 * We do NOT perform heavy processing here.
 *
 * We:
 *   1. Receive the frame
 *   2. Copy it into a queue
 *   3. Return
 *
 * The normal FreeRTOS task performs all decoding.
 * ============================================================ */

static bool can_rx_callback(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx
)
{
    (void)edata;
    (void)user_ctx;


    can_rx_message_t message;


    /*
     * Buffer used by the TWAI driver to place
     * the received CAN data.
     */

    twai_frame_t frame = {
        .buffer = message.data,
        .buffer_len = sizeof(message.data),
    };


    /*
     * Receive the frame.
     */

    if (
        twai_node_receive_from_isr(
            handle,
            &frame
        ) != ESP_OK
    )
    {
        return false;
    }


    /*
     * Save CAN identifier.
     */

    message.id =
        frame.header.id;


    /*
     * All current slave frames are
     * 8-byte classic CAN frames.
     */

    message.length =
        CAN_FRAME_LENGTH;


    /*
     * Put received frame into queue.
     */

    BaseType_t higher_priority_task_woken =
        pdFALSE;


    xQueueSendFromISR(
        can_rx_queue,
        &message,
        &higher_priority_task_woken
    );


    return (
        higher_priority_task_woken ==
        pdTRUE
    );
}


/* ============================================================
 * CAN INITIALIZATION
 * ============================================================ */

static void can_init(void)
{
    /*
     * Create receive queue first.
     */

    can_rx_queue =
        xQueueCreate(
            CAN_RX_QUEUE_LENGTH,
            sizeof(can_rx_message_t)
        );


    if (can_rx_queue == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create CAN RX queue"
        );

        abort();
    }


    /*
     * Configure on-chip TWAI.
     */

    twai_onchip_node_config_t config = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,

        .bit_timing.bitrate =
            CAN_BITRATE,

        /*
         * We are not transmitting from
         * the Master yet, but keeping a
         * small TX queue makes the node
         * ready for future commands.
         */

        .tx_queue_depth = 8,
    };


    ESP_ERROR_CHECK(
        twai_new_node_onchip(
            &config,
            &can_node
        )
    );


    /*
     * Register receive callback BEFORE
     * enabling the CAN node.
     */

    twai_event_callbacks_t callbacks = {
        .on_rx_done = can_rx_callback,
    };


    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(
            can_node,
            &callbacks,
            NULL
        )
    );


    /*
     * Enable CAN controller.
     */

    ESP_ERROR_CHECK(
        twai_node_enable(
            can_node
        )
    );


    ESP_LOGI(
        TAG,
        "CAN initialized"
    );

    ESP_LOGI(
        TAG,
        "CAN bitrate : %d bit/s",
        CAN_BITRATE
    );

    ESP_LOGI(
        TAG,
        "CAN TX      : GPIO %d",
        CAN_TX_GPIO
    );

    ESP_LOGI(
        TAG,
        "CAN RX      : GPIO %d",
        CAN_RX_GPIO
    );
}


/* ============================================================
 * SLAVE 1 DECODER
 * ============================================================ */

static void decode_slave1(
    const can_rx_message_t *message
)
{
    switch (message->id)
    {
        /*
         * ----------------------------------------------------
         * 0x100
         *
         * AX
         * AY
         * AZ
         * Sequence
         * ----------------------------------------------------
         */

        case S1_ID_IMU:
        {
            int16_t ax =
                read_be_signed16(
                    &message->data[0]
                );

            int16_t ay =
                read_be_signed16(
                    &message->data[2]
                );

            int16_t az =
                read_be_signed16(
                    &message->data[4]
                );

            uint16_t sequence =
                read_be16(
                    &message->data[6]
                );


            ESP_LOGI(
                TAG,
                "S1 | IMU | AX=%d AY=%d AZ=%d SEQ=%u",
                ax,
                ay,
                az,
                sequence
            );

            break;
        }


        /*
         * ----------------------------------------------------
         * 0x101
         *
         * GX
         * GY
         * GZ
         * Hall ADC
         * ----------------------------------------------------
         */

        case S1_ID_GYRO_HALL:
        {
            int16_t gx =
                read_be_signed16(
                    &message->data[0]
                );

            int16_t gy =
                read_be_signed16(
                    &message->data[2]
                );

            int16_t gz =
                read_be_signed16(
                    &message->data[4]
                );

            uint16_t hall_raw =
                read_be16(
                    &message->data[6]
                );


            ESP_LOGI(
                TAG,
                "S1 | GYRO/HALL | GX=%d GY=%d GZ=%d HALL=%u",
                gx,
                gy,
                gz,
                hall_raw
            );

            break;
        }


        /*
         * ----------------------------------------------------
         * 0x102
         *
         * Temp1 x100
         * Temp2 x100
         * Hall D0
         * RTD1 fault
         * RTD2 fault
         * Status
         * ----------------------------------------------------
         */

        case S1_ID_TEMP_STATUS:
        {
            int16_t temp1_x100 =
                read_be_signed16(
                    &message->data[0]
                );

            int16_t temp2_x100 =
                read_be_signed16(
                    &message->data[2]
                );

            uint8_t hall_d0 =
                message->data[4];

            uint8_t rtd1_fault =
                message->data[5];

            uint8_t rtd2_fault =
                message->data[6];

            uint8_t status =
                message->data[7];


            ESP_LOGI(
                TAG,
                "S1 | TEMP | T1=%.2f C T2=%.2f C HALL_D0=%u RTD1F=%u RTD2F=%u STATUS=0x%02X",
                temp1_x100 / 100.0f,
                temp2_x100 / 100.0f,
                hall_d0,
                rtd1_fault,
                rtd2_fault,
                status
            );

            break;
        }


        default:
            break;
    }
}


/* ============================================================
 * SLAVE 2 DECODER
 * ============================================================ */

static void decode_slave2(
    const can_rx_message_t *message
)
{
    if (message->id == S2_ID_LIDAR)
    {
        int16_t distance_mm = read_be_signed16(&message->data[0]);
        uint16_t sequence = read_be16(&message->data[2]);
        uint8_t status = message->data[4];

        ESP_LOGI(TAG, "S2 | LIDAR | DIST=%d mm STATUS=0x%02X SEQ=%u",
                 distance_mm, status, sequence);
        return;
    }

    if (message->id != S2_ID_ADC)
        return;


    /*
     * 0x200
     *
     * Byte 0-1:
     *     Raw ADC
     *
     * Byte 2-3:
     *     ADC voltage mV
     *
     * Byte 4-5:
     *     Input voltage mV
     *
     * Byte 6-7:
     *     Packet counter
     */

    int16_t raw =
        read_be_signed16(
            &message->data[0]
        );


    uint16_t adc_mv =
        read_be16(
            &message->data[2]
        );


    uint16_t input_mv =
        read_be16(
            &message->data[4]
        );


    uint16_t sequence =
        read_be16(
            &message->data[6]
        );


    ESP_LOGI(
        TAG,
        "S2 | ADC | RAW=%d ADC=%.3f V INPUT=%.3f V SEQ=%u",
        raw,
        adc_mv / 1000.0f,
        input_mv / 1000.0f,
        sequence
    );
}


/* ============================================================
 * SLAVE 3 DECODER
 * ============================================================ */

static void decode_slave3(
    const can_rx_message_t *message
)
{
    switch (message->id)
    {
        /*
         * ----------------------------------------------------
         * 0x300
         *
         * T1 ambient
         * T1 object
         * T2 ambient
         * T2 object
         * ----------------------------------------------------
         */

        case S3_ID_TEMPERATURE:
        {
            int16_t t1_ambient =
                read_be_signed16(
                    &message->data[0]
                );

            int16_t t1_object =
                read_be_signed16(
                    &message->data[2]
                );

            int16_t t2_ambient =
                read_be_signed16(
                    &message->data[4]
                );

            int16_t t2_object =
                read_be_signed16(
                    &message->data[6]
                );


            ESP_LOGI(
                TAG,
                "S3 | TEMP | T1A=%.2f C T1O=%.2f C T2A=%.2f C T2O=%.2f C",
                t1_ambient / 100.0f,
                t1_object / 100.0f,
                t2_ambient / 100.0f,
                t2_object / 100.0f
            );

            break;
        }


        /*
         * ----------------------------------------------------
         * 0x301
         *
         * Byte 0-1:
         *     LiDAR mm
         *
         * Byte 2-5:
         *     Pressure Pa
         *
         * Byte 6-7:
         *     Sequence
         * ----------------------------------------------------
         */

        case S3_ID_DISTANCE_PRESSURE:
        {
            uint16_t lidar_mm =
                read_be16(
                    &message->data[0]
                );

            uint32_t pressure_pa =
                read_be32(
                    &message->data[2]
                );

            uint16_t sequence =
                read_be16(
                    &message->data[6]
                );


            ESP_LOGI(
                TAG,
                "S3 | DIST/PRESS | LIDAR=%u mm PRESSURE=%lu Pa SEQ=%u",
                (unsigned int)lidar_mm,
                (unsigned long)pressure_pa,
                sequence
            );

            break;
        }


        /*
         * ----------------------------------------------------
         * 0x302
         *
         * Status byte
         * ----------------------------------------------------
         */

        case S3_ID_STATUS:
        {
            uint8_t status =
                message->data[0];


            ESP_LOGI(
                TAG,
                "S3 | STATUS | 0x%02X | T1=%s T2=%s LIDAR=%s PRESSURE=%s",
                status,
                (status & 0x01) ? "OK" : "ERR",
                (status & 0x02) ? "OK" : "ERR",
                (status & 0x04) ? "OK" : "ERR",
                (status & 0x08) ? "OK" : "ERR"
            );

            break;
        }


        default:
            break;
    }
}


/* ============================================================
 * SLAVE 4 DECODER
 * ============================================================ */

static void decode_slave4(
    const can_rx_message_t *message
)
{
    switch (message->id)
    {
        /*
         * ----------------------------------------------------
         * 0x400
         *
         * PT100 #1 temperature x100
         * PT100 #2 temperature x100
         * status
         * packet counter
         * ----------------------------------------------------
         */

        case S4_ID_RTDS:
        {
            int16_t temp1_x100 =
                read_be_signed16(
                    &message->data[0]
                );

            int16_t temp2_x100 =
                read_be_signed16(
                    &message->data[2]
                );

            uint8_t status =
                message->data[4];

            uint16_t sequence =
                read_be16(
                    &message->data[5]
                );


            ESP_LOGI(
                TAG,
                "S4 | RTD | T1=%.2f C T2=%.2f C STATUS=0x%02X SEQ=%u",
                temp1_x100 / 100.0f,
                temp2_x100 / 100.0f,
                status,
                sequence
            );

            break;
        }


        /*
         * ----------------------------------------------------
         * 0x401
         *
         * Hall raw
         * Hall voltage mV
         * status
         * packet counter
         * ----------------------------------------------------
         */

        case S4_ID_HALL:
        {
            uint16_t hall_raw =
                read_be16(
                    &message->data[0]
                );

            uint16_t hall_mv =
                read_be16(
                    &message->data[2]
                );

            uint8_t status =
                message->data[4];

            uint16_t sequence =
                read_be16(
                    &message->data[5]
                );


            ESP_LOGI(
                TAG,
                "S4 | HALL | RAW=%u V=%.3f V STATUS=0x%02X SEQ=%u",
                hall_raw,
                hall_mv / 1000.0f,
                status,
                sequence
            );

            break;
        }


        default:
            break;
    }
}


/* ============================================================
 * GENERAL CAN DECODER
 * ============================================================ */

static void process_can_message(
    const can_rx_message_t *message
)
{
    switch (message->id)
    {
        /*
         * Slave 1
         */

        case S1_ID_IMU:
        case S1_ID_GYRO_HALL:
        case S1_ID_TEMP_STATUS:

            slave1_last_rx =
                xTaskGetTickCount();

            decode_slave1(message);

            break;


        /*
         * Slave 2
         */

        case S2_ID_ADC:
        case S2_ID_LIDAR:

            slave2_last_rx =
                xTaskGetTickCount();

            decode_slave2(message);

            break;


        /*
         * Slave 3
         */

        case S3_ID_TEMPERATURE:
        case S3_ID_DISTANCE_PRESSURE:
        case S3_ID_STATUS:

            slave3_last_rx =
                xTaskGetTickCount();

            decode_slave3(message);

            break;


        /*
         * Slave 4
         */

        case S4_ID_RTDS:
        case S4_ID_HALL:

            slave4_last_rx =
                xTaskGetTickCount();

            decode_slave4(message);

            break;


        /*
         * Unknown CAN ID
         */

        default:

            ESP_LOGW(
                TAG,
                "UNKNOWN CAN ID: 0x%03lX",
                (unsigned long)message->id
            );

            break;
    }

    /* Decode first, then publish the matching Master telemetry frame. */
    if (master_telemetry_id(message->id) != 0)
        publish_master_telemetry(message);
}


/* ============================================================
 * CAN PROCESSING TASK
 * ============================================================ */

static void can_processing_task(
    void *arg
)
{
    (void)arg;


    can_rx_message_t message;


    while (1)
    {
        /*
         * Wait for a received CAN frame.
         */

        if (
            xQueueReceive(
                can_rx_queue,
                &message,
                portMAX_DELAY
            ) == pdTRUE
        )
        {
            process_can_message(
                &message
            );
        }
    }
}


/* ============================================================
 * MASTER STATUS TASK
 *
 * This is only monitoring for now.
 *
 * Timeout:
 * 2 seconds without receiving any
 * frame from a slave = OFFLINE.
 * ============================================================ */

static void master_status_task(
    void *arg
)
{
    (void)arg;


    const TickType_t timeout =
        pdMS_TO_TICKS(2000);


    while (1)
    {
        TickType_t now =
            xTaskGetTickCount();


        bool slave1_online =
            (now - slave1_last_rx) <
            timeout;

        bool slave2_online =
            (now - slave2_last_rx) <
            timeout;

        bool slave3_online =
            (now - slave3_last_rx) <
            timeout;

        bool slave4_online =
            (now - slave4_last_rx) <
            timeout;


        ESP_LOGI(
            TAG,
            "STATUS | S1=%s S2=%s S3=%s S4=%s",
            slave1_online ? "ONLINE" : "OFFLINE",
            slave2_online ? "ONLINE" : "OFFLINE",
            slave3_online ? "ONLINE" : "OFFLINE",
            slave4_online ? "ONLINE" : "OFFLINE"
        );

        ESP_LOGI(TAG, "MASTER TELEMETRY TX | OK=%" PRIu32 " FAIL=%" PRIu32,
                 master_tlm_tx_ok, master_tlm_tx_fail);

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "================================================"
    );

    ESP_LOGI(
        TAG,
        "           ESP MULTI SENSOR MASTER"
    );

    ESP_LOGI(
        TAG,
        "================================================"
    );

    ESP_LOGI(
        TAG,
        "CAN bitrate : 500 kbit/s"
    );

    ESP_LOGI(
        TAG,
        "CAN TX      : GPIO 5"
    );

    ESP_LOGI(
        TAG,
        "CAN RX      : GPIO 4"
    );

    ESP_LOGI(
        TAG,
        "Encoding    : BIG-ENDIAN"
    );


    /*
     * Initialize CAN.
     */

    can_init();


    /*
     * Start CAN processing task.
     */

    xTaskCreate(
        can_processing_task,
        "can_processing",
        4096,
        NULL,
        10,
        NULL
    );


    /*
     * Start Master status task.
     */

    xTaskCreate(
        master_status_task,
        "master_status",
        3072,
        NULL,
        5,
        NULL
    );


    ESP_LOGI(
        TAG,
        "Master initialized."
    );

    ESP_LOGI(
        TAG,
        "Waiting for Slave 1, Slave 2, Slave 3 and Slave 4..."
    );
}
