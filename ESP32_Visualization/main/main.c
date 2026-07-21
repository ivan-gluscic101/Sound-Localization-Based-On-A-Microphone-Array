#include <stdio.h>
#include "esp_log.h"
#include "uart_driver.h"
#include "sound_loc_processor.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "nvs_flash.h"

static const char *TAG = "SOUND_LOC_ESP32";

/* Entry point: init NVS, Wi-Fi AP, web server, UART and packet processor. */
void app_main(void) {
    /* NVS init (required for Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Inicijalizacija sustava...");
    
    wifi_manager_init();
    web_server_init();
    uart_driver_init();
    sound_loc_processor_init();
    ESP_LOGI(TAG, "Sustav spreman.");
}