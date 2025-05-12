#include <string.h>
#include "uart_handler.h"
#include "light_control.h"
#include "pwm_control.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"

#define UART_NUM UART_NUM_1 // SET UART_1
#define BUF_SIZE 256
#define TASK_STACK_SIZE 4096
#define UART_RX_PIN 4 //UART Rx GPIO PIN
#define UART_TX_PIN 5 //UART Tx GPIO PIN

static const char *TAG = "uart_handler";
static QueueHandle_t uart_queue;
static void uart_event_task(void *arg);

// UART Init Cofigure
esp_err_t uart_init(void) 
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

// Send UART Acknowladge 
void uart_send_ack(uint16_t dimming, uint16_t tuning)
{
    cJSON *ack_json = cJSON_CreateObject();
    if (ack_json == NULL) 
        return;

    cJSON_AddNumberToObject(ack_json, "dimming", dimming);
    cJSON_AddNumberToObject(ack_json, "tuning", tuning);
    cJSON_AddStringToObject(ack_json, "status", "ack");

    char *ack_str = cJSON_PrintUnformatted(ack_json);
    if (ack_str == NULL) {
        cJSON_Delete(ack_json);
        return;
    }

    uart_write_bytes(UART_NUM, ack_str, strlen(ack_str));
    
    cJSON_Delete(ack_json);
    cJSON_free(ack_str);
}

void uart_start(void) // UART Task Creation
{
    xTaskCreate(uart_event_task, "uart_event_task", TASK_STACK_SIZE, NULL, 12, NULL);
}

static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint16_t* data = (uint16_t*) malloc(BUF_SIZE);

    for (   ;   ;   ) 
    {
        // Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY) == pdTRUE) 
        {
            bzero(data, BUF_SIZE);
            switch (event.type) 
            {
                case UART_DATA:
                    uart_read_bytes(UART_NUM, data, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "data %s", (char *)data);
                    cJSON *json = cJSON_Parse((char *)data);
                    if (json) 
                    {
                        cJSON *dimming = cJSON_GetObjectItem(json, "dimming");
                        cJSON *tuning  = cJSON_GetObjectItem(json, "tuning");

                        int flag = 0;
                        light_data_t light_data;

                        if (cJSON_IsNumber(dimming)) 
                        {
                            uint16_t dim = dimming->valueint;
                            if (dim <= 100) 
                            {
                                light_data.dimming = dim;
                                flag = 1;
                            } 
                            else 
                            {
                                ESP_LOGW(TAG, "Dimming value out of range");
                            }
                        }

                        if (cJSON_IsNumber(tuning)) 
                        {
                            uint16_t tun = tuning->valueint;
                            if (tun <= 100) 
                            {
                                light_data.tuning = tun;
                                flag = 1;
                            } 
                            else 
                            {
                                ESP_LOGW(TAG, "Tuning value out of range");
                            }
                        }

                        if (flag) // Send Data into Light Queue
                        {
                            if (send_to_light_queue(&light_data) == ESP_OK)
                            {
                                ESP_LOGI(TAG, "Queued dimming=%d, tuning=%d", light_data.dimming, light_data.tuning);
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Queue full. Data not sent.");
                            }
                        } 
                        else 
                        {
                            ESP_LOGW(TAG, "No valid dimming or tuning values found");
                        }

                        cJSON_Delete(json);
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "JSON parse error");
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO Overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring Buffer Full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                
                default:
                    ESP_LOGI(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}
