// sensor_uart.c

#include <stdio.h>
#include "esp_err.h"
#include "sensor_uart.h"
#include "driver/uart.h"
#include "string.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define UART_SENSOR_NUM UART_NUM_0
#define BUF_SIZE 1024
#define UART_RX_PIN 7
#define UART_TX_PIN 10

extern QueueHandle_t presence_queue;
extern config_t global_config;
static QueueHandle_t sensor_event_queue;

static const char *TAG = "uart_sensor";

// static const char SETTINGS_OPEN[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00,0x01,0x00, 0x04, 0x03, 0x02, 0x01};
// static const char SETTINGS_CLOSE[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
// static const char FETCH_RANGE[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
// static char SET_RANGE[18] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x07, 0x00, 0x01, 0x00, 0x09, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

void sensor_uart_config(sensor_config_t *config)
 {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "SET_RANGE:%d\n", config->range);
    uart_write_bytes(UART_SENSOR_NUM, cmd, strlen(cmd));

    snprintf(cmd, sizeof(cmd), "SET_TIMEOUT:%d\n", config->timeout);
    uart_write_bytes(UART_SENSOR_NUM, cmd, strlen(cmd));
}

void sensor_uart_task(void *pvParameters) 
{
    uart_event_t event;
    uint8_t data[BUF_SIZE];
    while (1) 
    {
       if(xQueueReceive(sensor_event_queue, &event, portMAX_DELAY) == pdPASS)
       {
            if(event.type)
            {
                int len = uart_read_bytes(UART_SENSOR_NUM, data, event.size, portMAX_DELAY);
                data[len] = 0;
                ESP_LOGI(TAG, "Sensore UART Receive: %s", data);
                if(strstr((char *)data, "PRESENCE_DETECTED"))
                {
                    int evt = 1;
                    xQueueSend(presence_queue, &evt, portMAX_DELAY);
                }
                else if(strstr((char *)data, "VACANT"))
                {
                    int evt = 0;
                    xQueueSend(presence_queue, &evt, portMAX_DELAY);
                }
                
            }
       }
    }
}

esp_err_t sensor_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    esp_err_t ret = uart_param_config(UART_SENSOR_NUM, &uart_config);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_SENSOR_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;   
    }

    ret = uart_driver_install(UART_SENSOR_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}