#include "pwm_control.h"
#include "light_control.h"
#include "nvs_storage.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_handler.h"

#define LEDC_TIMER              LEDC_TIMER_0 // Set LEDC Timer_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE // Set LEDC Low Speed Mode
#define LEDC_FREQ_HZ            1000 // Frequncy 1000 
#define LEDC_RESOLUTION         LEDC_TIMER_10_BIT // Set 10 bit Resolution
#define DIMMING_CHANNEL         LEDC_CHANNEL_0 // Channel_0 for dimming
#define TUNING_CHANNEL          LEDC_CHANNEL_1 // channel_1 for tuning
#define DIMMING_GPIO            4 
#define TUNING_GPIO             5
#define TASK_STACK_SIZE         2048
#define LIGHT_QUEUE_LENGTH      10

QueueHandle_t light_queue = NULL;

static const char *TAG = "pwm_control";

static void pwm_control_task(void *arg);

// Set PWM & LEDC Timer Parameter and Configure
esp_err_t pwm_control_init(void)
{
    light_control_queue_init();
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t channels[2] = {
        {
            .speed_mode     = LEDC_MODE,
            .channel        = DIMMING_CHANNEL,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = DIMMING_GPIO,
            .duty           = 0,
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = TUNING_CHANNEL,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = TUNING_GPIO,
            .duty           = 0,
            .hpoint         = 0
        }
    };

    for (int i = 0; i < 2; i++) 
    {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }

    // Restore saved values
    light_data_t saved = {0};
    if (nvs_storage_read(&saved) == ESP_OK) 
    {
        ESP_LOGI(TAG, "Restored dimming=%d tuning=%d from NVS", saved.dimming, saved.tuning);
        ledc_set_duty(LEDC_MODE, DIMMING_CHANNEL, (saved.dimming * 1023) / 100);
        ledc_update_duty(LEDC_MODE, DIMMING_CHANNEL);
        ledc_set_duty(LEDC_MODE, TUNING_CHANNEL, (saved.tuning * 1023) / 100);
        ledc_update_duty(LEDC_MODE, TUNING_CHANNEL);
    }
    else
        ESP_LOGW(TAG, "No saved values in NVS");

    return ESP_OK;
}

// PWM Control Task Creation
void pwm_control_start(void)
{
    xTaskCreate(pwm_control_task, "pwm_ctrl_task", TASK_STACK_SIZE, NULL, 1, NULL);
}

static void pwm_control_task(void *arg)
{
    light_data_t received;

    while (1) 
    {
        if(light_queue != NULL)

        if (xQueueReceive(light_queue, &received, portMAX_DELAY) == pdTRUE) // Check data is received 
        {
            // Convert in percentage to duty cycle
            uint16_t dim_duty = (received.dimming * 1023) / 100;
            uint16_t tun_duty = (received.tuning * 1023) / 100;

            ESP_LOGI(TAG, "Update dimming=%d, tuning=%d", received.dimming, received.tuning);
            
            // Set and Update new duty cycle value in GPIO
            if (ledc_set_duty(LEDC_MODE, DIMMING_CHANNEL, dim_duty) != ESP_OK)
                ESP_LOGE(TAG, "Failed to ledc set duty dimming");

            if (ledc_update_duty(LEDC_MODE, DIMMING_CHANNEL) != ESP_OK)
                ESP_LOGE(TAG, "Failed to ledc uodate duty dimming");

            if (ledc_set_duty(LEDC_MODE, TUNING_CHANNEL, tun_duty) != ESP_OK)
                ESP_LOGE(TAG, "Failed to ledc set duty tuning");

            if (ledc_update_duty(LEDC_MODE, TUNING_CHANNEL) != ESP_OK)
                ESP_LOGE(TAG, "Failed to ledc set duty tuning");

            if (nvs_storage_write(&received) != ESP_OK) // Save New value in NVS
                ESP_LOGE(TAG, "Failed to save values to NVS");

            uart_send_ack(received.dimming, received.tuning);
        }
    }
}

esp_err_t light_control_queue_init(void) // Task Creation
{
    light_queue = xQueueCreate(LIGHT_QUEUE_LENGTH, sizeof(light_data_t));
    if (light_queue == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create light queue");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Light queue created successfully");
    return ESP_OK;
}

esp_err_t send_to_light_queue(const light_data_t *data) //Send light data in Queue
{
    if (data == NULL) 
    {
        ESP_LOGE(TAG, "Null data passed to send_to_light_queue");
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueSend(light_queue, data, portMAX_DELAY) != pdPASS) 
    {
        ESP_LOGE(TAG, "Failed to send data to light_queue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Data sent to light_queue successfully");
    return ESP_OK;
}
