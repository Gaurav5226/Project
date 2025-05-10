#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"

#include "uart_handler.h"
#include "pwm_control.h"
#include "nvs_storage.h"
#include "light_control.h"

static const char *TAG = "main";
extern QueueHandle_t light_queue;
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Light Control Cmd Firmware");

    esp_err_t err = nvs_storage_init(); // NVS
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed NVS: %s", esp_err_to_name(err));
        return;
    }

    err = pwm_control_init(); // PWM module
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to PWM: %s", esp_err_to_name(err));
        return;
    }

    err = uart_init(); // UART
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize UART: %s", esp_err_to_name(err));
        return;
    }

    // UART and PWM start
    pwm_control_start();
    uart_start();
    

    ESP_LOGI(TAG, "System successfully Run");
}
