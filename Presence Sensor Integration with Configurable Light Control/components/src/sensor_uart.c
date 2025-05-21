// sensor_uart.c

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "sensor_uart.h"
#include "driver/uart.h"
#include "pwm_control.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "string.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <inttypes.h>


#define UART_SENSOR_NUM      UART_NUM_1
#define BUF_SIZE             1024
#define UART_RX_PIN          GPIO_NUM_10
#define UART_TX_PIN          GPIO_NUM_7
#define GPIO_INPUT_IO        GPIO_NUM_3
#define GPIO_INPUT_PIN_SEL   (1ULL << GPIO_INPUT_IO)
#define ESP_INTR_FLAG_DEFAULT 0
#define LED_TIMEOUT_MS 30000
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0


const char SETTINGS_OPEN[14]  = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
const char SETTINGS_CLOSE[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
const char FETCH_RANGE[14]    = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
char SET_RANGE[18]            = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x07, 0x00, 0x01, 0x00, 0x09, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

//extern QueueHandle_t presence_queue;
// extern sensor_config_t global_config;
QueueHandle_t sensor_event_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;
static TimerHandle_t led_timer = NULL;

static const char *TAG = "uart_sensor";

// ISR handler for GPIO interrupt
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Timer callback to turn off LED
static void led_timer_callback(TimerHandle_t xTimer)
{
    int state = 0; // LED OFF
    xQueueSend(pwm_control_get_presence_queue(), &state, 0);
    ESP_LOGI(TAG, "LED turned OFF due to timeout");
}

// Task to handle GPIO events
static void gpio_event_task(void* arg)
{
    uint8_t io_num;
    for(;;) 
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int level = gpio_get_level(io_num);
            ESP_LOGI(TAG, "GPIO Interrupt on GPIO: %d, level: %d", io_num, level);

            int state = (level == 1) ? 1 : 0;  // LED ON if motion detected, else OFF

            if(state == 1) {
                // Motion detected - stop the timer if running
                if (led_timer != NULL) {
                    esp_timer_stop(led_timer);
                }
                xQueueSend(pwm_control_get_presence_queue(), &state, 0);
            } 
            else {
                // No motion - start the timer to turn off LED after timeout
                if (led_timer != NULL) {
                    esp_timer_stop(led_timer);
                    esp_timer_start_once(led_timer, global_config.timeout * 1000); // Convert ms to us
                }
            }
        }
    }
}


void sensor_uart_config(sensor_config_t *config) 
{
    // Send SETTINGS_OPEN command
    uart_write_bytes(UART_SENSOR_NUM, SETTINGS_OPEN, sizeof(SETTINGS_OPEN));

    // Prepare and send SET_RANGE command
    SET_RANGE[12] = config->range;
    uart_write_bytes(UART_SENSOR_NUM, SET_RANGE, sizeof(SET_RANGE));

    // Send FETCH_RANGE command
    uart_write_bytes(UART_SENSOR_NUM, FETCH_RANGE, sizeof(FETCH_RANGE));

    // Send SETTINGS_CLOSE command
    uart_write_bytes(UART_SENSOR_NUM, SETTINGS_CLOSE, sizeof(SETTINGS_CLOSE));
}

void sensor_uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[BUF_SIZE];
    //QueueHandle_t presence_queue = pwm_control_get_presence_queue();
    while (1) 
    {
        if(sensor_event_queue != NULL)
        {
            if (xQueueReceive(sensor_event_queue, &event, portMAX_DELAY) == pdPASS) 
            {
                switch(event.type)
                {
                    case UART_DATA:
                        int len = uart_read_bytes(UART_SENSOR_NUM, data, event.size, pdMS_TO_TICKS(100));
                        data[len] = 0;

                        if (data[0] == 0xFD && data[1] == 0xFC && data[2] == 0xFB && data[3] == 0xFA && data[len - 4] == 0x04 && data[len - 3] == 0x03 && data[len - 2] == 0x02 && data[len - 1] == 0x01)
                        {
                            ESP_LOGI(TAG, "Sensor UART Received: %s", data);
                            ESP_LOG_BUFFER_HEX(TAG, data, len);
                        }
                        break;
                        
                    case UART_FIFO_OVF:
                        ESP_LOGW(TAG, "HW FIFO Overflow");
                        uart_flush_input(UART_SENSOR_NUM);
                        xQueueReset(pwm_control_get_presence_queue());
                        break;

                    case UART_BUFFER_FULL:
                        ESP_LOGW(TAG, "Ring Buffer Full");
                        uart_flush_input(UART_SENSOR_NUM);
                        xQueueReset(pwm_control_get_presence_queue());
                        break;
                    
                    default:
                        ESP_LOGI(TAG, "UART event type: %d", event.type);
                        break;
                }
            }
            else
            {
                ESP_LOGI(TAG, "Queue is not received");
                if (led_timer != NULL && xTimerIsTimerActive(led_timer) == pdFALSE) {
                    xTimerStart(led_timer, 0);
                }
            }
        }
        else
        {
            ESP_LOGI(TAG, "sensor_event_queue is null");
        }
    }
}

esp_err_t sensor_uart_init(void) 
{
    ESP_LOGI(TAG, "Initializing sensor UART");

    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    esp_err_t ret = uart_param_config(UART_SENSOR_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_SENSOR_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(UART_SENSOR_NUM, BUF_SIZE * 2, 0, 20, &sensor_event_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create UART task
    BaseType_t xReturned;
    xReturned = xTaskCreate(sensor_uart_task, "sensor_uart_task", 4096, NULL, 2, NULL);
    if(xReturned == pdPASS)
    {
        ESP_LOGI(TAG, "sensor_uart_task created");
    }

    // Configure GPIO for interrupt
    gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_ANYEDGE,  // Trigger on both rising and falling edges
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INPUT_PIN_SEL,
    .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);


    // Create a queue to handle GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if(gpio_evt_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create gpio_evt_queue");
        return ESP_FAIL;
    }

    // Start a task to handle GPIO events
    xReturned = xTaskCreate(gpio_event_task, "gpio_event_task", 2048, NULL, 10, NULL);
    if(xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create gpio_event_task");
        return ESP_FAIL;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Hook ISR handler for specific GPIO pin
    ret = gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create LED timer using esp_timer
    const esp_timer_create_args_t led_timer_args = {
        .callback = &led_timer_callback,
        .name = "led_timer"
    };

    ret = esp_timer_create(&led_timer_args, &led_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED esp_timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "sensor_uart_init completed successfully");

    return ESP_OK;
}
