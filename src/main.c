#include "esp_log.h"
#include "nmea_parser.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "icfm";

void app_main() {
    esp_log_level_set("*", ESP_LOG_INFO);
    int counter = 0;
    for (;;) {
        ESP_LOGI(TAG, "Hello, World! number %d", counter);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        counter++;
    }
}