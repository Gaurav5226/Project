// uart_pc.c

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "uart_pc.h"
#include "pwm_control.h"
#include "nvs_storage.h"
#include "sensor_uart.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"

#define UART_PC_NUM UART_NUM_0       // Use UART0
#define BUF_SIZE 1024                // Buffer size for UART data
#define TASK_STACK_SIZE 4096         // Stack size for the UART task
#define UART_RX_PIN 20               // UART RX GPIO
#define UART_TX_PIN 21               // UART TX GPIO

static const char *TAG = "uart_pc";  // Logging tag
static QueueHandle_t uart_event_queue;  // UART event queue handle

// Function to parse and handle incoming JSON command
static void handle_json_command(const char *input)
{
    ESP_LOGI(TAG, "Parsing JSON command: %s", input);

    cJSON *root = cJSON_Parse(input);
    if (!root) 
    {
        ESP_LOGE(TAG, "JSON Parse Failed");
        return;
    }

    // Handle "get_config" command
    cJSON *command = cJSON_GetObjectItem(root, "command");
    if (command && cJSON_IsString(command) && strcmp(command->valuestring, "get_config") == 0)
    {
        ESP_LOGI(TAG,"Range :%d", global_config.range);
        ESP_LOGI(TAG,"on_level :%d", global_config.on_level);

        cJSON *response = cJSON_CreateObject();
        cJSON *sensor = cJSON_CreateObject();
        cJSON *light = cJSON_CreateObject();

        cJSON_AddNumberToObject(sensor, "range", global_config.range);
        cJSON_AddNumberToObject(sensor, "timeout", global_config.timeout);
        cJSON_AddNumberToObject(light, "on_level", global_config.on_level);
        cJSON_AddNumberToObject(light, "off_level", global_config.off_level);

        cJSON_AddItemToObject(response, "sensor", sensor);
        cJSON_AddItemToObject(response, "light", light);

        char *json_str = cJSON_PrintUnformatted(response);
        if (json_str)
        {
            // Send JSON response over UART
            uart_write_bytes(UART_PC_NUM, json_str, strlen(json_str));
            uart_write_bytes(UART_PC_NUM, "\n", 1);
            ESP_LOGI(TAG, "Sent config JSON: %s", json_str);
            free(json_str);
        }
        cJSON_Delete(response);
    } 
    else 
    {
        // Handle incoming config JSON to update sensor or light config
        cJSON *sensor = cJSON_GetObjectItem(root, "sensor");
        cJSON *light = cJSON_GetObjectItem(root, "light");

        if (sensor && cJSON_IsObject(sensor)) 
        {
            cJSON *range_item = cJSON_GetObjectItem(sensor, "range");
            cJSON *timeout_item = cJSON_GetObjectItem(sensor, "timeout");

            if (cJSON_IsNumber(range_item)) {
                global_config.range = range_item->valueint;
                sensor_uart_config(&global_config);  // Update motion range
            }
            if (cJSON_IsNumber(timeout_item)) {
                global_config.timeout = timeout_item->valueint;
            }

            // Save updated config to NVS
            nvs_save_config(&global_config);
            ESP_LOGI(TAG, "Sensor config updated and saved");
        }
        
        if (light && cJSON_IsObject(light))
        {
            cJSON *on_level_item = cJSON_GetObjectItem(light, "on_level");
            cJSON *off_level_item = cJSON_GetObjectItem(light, "off_level");

            if (cJSON_IsNumber(on_level_item)) {
                global_config.on_level = on_level_item->valueint;
            }
            if (cJSON_IsNumber(off_level_item)) {
                global_config.off_level = off_level_item->valueint;
            }

            // Apply new brightness levels
            pwm_control_set_levels(global_config.on_level, global_config.off_level);
            ESP_LOGI(TAG, "Light config updated and saved");
        }
        else
        {
            ESP_LOGW(TAG, "Invalid or missing 'sensor' object in JSON");
        }
    }

    cJSON_Delete(root);
}

// UART Task to handle events and process incoming data
void uart_pc_task(void *pvParameters) 
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);  // Buffer to hold received data

    while (1) 
    {
        // Wait for UART event from queue
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY) == pdPASS) 
        {
            switch(event.type) 
            {
                case UART_DATA:
                    // Data received from UART
                    int len = uart_read_bytes(UART_PC_NUM, data, event.size, portMAX_DELAY);
                    data[len] = 0;  // Null-terminate for string parsing
                    ESP_LOGI(TAG, "UART Received: %s", data);
                    handle_json_command((char *)data);  // Parse and handle JSON
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO Overflow");
                    uart_flush_input(UART_PC_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring Buffer Full");
                    uart_flush_input(UART_PC_NUM);
                    xQueueReset(uart_event_queue);
                    break;
                
                default:
                    ESP_LOGI(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "Queue is not received");
        }
    }

    free(data);
}

// UART initialization and task creation
esp_err_t uart_pc_init(void) 
{
    ESP_LOGI(TAG, "uart_pc_init");

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    // Apply UART configuration
    esp_err_t ret = uart_param_config(UART_PC_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART TX and RX pins
    ret = uart_set_pin(UART_PC_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install UART driver and create event queue
    ret = uart_driver_install(UART_PC_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_event_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create task to handle UART data
    BaseType_t xReturned = xTaskCreate(uart_pc_task, "uart_pc_task", TASK_STACK_SIZE, NULL, 3, NULL);
    if(xReturned == pdPASS)
    {
        ESP_LOGI(TAG, "UART task created successfully");
    }

    return ESP_OK;
}
