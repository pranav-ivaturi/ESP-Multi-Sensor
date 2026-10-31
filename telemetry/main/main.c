
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TELEMETRY";

void app_main(void)
{
    ESP_LOGI(TAG, "Telemetry system initialized");

    while (1) {
        ESP_LOGI(TAG, "Telemetry running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
