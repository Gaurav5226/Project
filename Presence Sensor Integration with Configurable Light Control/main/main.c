// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "uart_pc.h"
#include "sensor_uart.h"
#include "pwm_control.h"
#include "nvs_storage.h"
#include "config.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing....");

    // Initialize NVS — required for storing persistent data
    esp_err_t ret = nvs_flash_init();
    ESP_LOGI(TAG, "nvs_flash_init returned err = %s", esp_err_to_name(ret));

    // Handle case where NVS partition is truncated and must be erased
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) 
    {
        ESP_LOGW(TAG, "NVS has no free pages, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();  // Retry initialization after erase
    }
    ESP_ERROR_CHECK(ret);

    // Initialize
    nvs_storage_init();   // Load saved settings from NVS or initialize defaults
    pwm_control_init();   // Set up PWM channels and control structures
    uart_pc_init();       // Initialize UART communication for receiving JSON commands
    sensor_uart_init();   // Configure UART/GPIO for motion detection and sensor control
}