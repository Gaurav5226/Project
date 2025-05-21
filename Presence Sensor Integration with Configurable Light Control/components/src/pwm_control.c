#include "pwm_control.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include <inttypes.h> 

#define LED_GPIO        1
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ    1000

QueueHandle_t presence_queue;
// extern sensor_config_t global_config/;

static const char *TAG = "pwm_control";

// Function to set PWM levels for ON and OFF states
void pwm_control_set_levels(int on_percent, int off_percent)
{
    global_config.on_level = on_percent;
    global_config.off_level = off_percent;

    ESP_LOGI(TAG, "PWM levels: ON=%d%%, OFF=%d%%", global_config.on_level, global_config.off_level);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Task to handle PWM control based on sensor presence state
static void pwm_control_task(void *pvParameters)
{
    int presence_state = 0;
    while (1)
    {
        if (xQueueReceive(presence_queue, &presence_state, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "Presence state received: %d", presence_state);

            // Determine the target duty cycle
            uint32_t target_duty = (presence_state == 0) ? global_config.off_level : global_config.on_level ;
            
            target_duty = (target_duty * 1023)/100;
            ESP_LOGI(TAG, "Target Duty: %" PRIu32, target_duty);

            // Check if a fade operation is already in progress
            if (ledc_get_duty(LEDC_MODE, LEDC_CHANNEL) != target_duty)
            {
                // Fade with time
                esp_err_t err = ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, target_duty, 100);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "ledc_set_fade_with_time failed: %s", esp_err_to_name(err));
                }

                err = ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_WAIT_DONE);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "ledc_fade_start failed: %s", esp_err_to_name(err));
                }
            }
            else
            {
                ESP_LOGI(TAG, "Fade operation already in progress");
            }
        }
        else
        {
            ESP_LOGI(TAG, "Queue Receive failed");
        }
    }
}


// Function to initialize PWM control
esp_err_t pwm_control_init(void)
{
    // Timer config
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Channel config
    ledc_channel_config_t channel = {
        .gpio_num       = LED_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .intr_type      = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    // Default setting
    // sensor_config_t global_config = {
    // .range = 0.5,
    // .timeout = 3000,
    // .on_level = 90,  // 80% duty cycle
    // .off_level = 10  // 10% duty cycle
    // };

    if (global_config.on_level == 0 && global_config.off_level == 0) 
    {
        global_config.on_level = 90;
        global_config.off_level = 10;
    }


    // Install fade service
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    // Create queue
    presence_queue = xQueueCreate(10, sizeof(int));
    if (presence_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create presence queue");
        return ESP_FAIL;
    }

    // Set default levels and start control task
    pwm_control_set_levels(global_config.on_level, global_config.off_level);
    BaseType_t result = xTaskCreate(pwm_control_task, "pwm_control_task", 4096, NULL, 1, NULL);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create pwm control task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

QueueHandle_t pwm_control_get_presence_queue(void)
{
    return presence_queue;
}
