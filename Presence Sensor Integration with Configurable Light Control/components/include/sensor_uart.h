// sensor_uart.h
#ifndef SENSOR_UART_H
#define SENSOR_UART_H

#include "config.h"

esp_err_t sensor_uart_init(void);
// void sensor_uart_task(void *pvParameters);
void sensor_uart_config(sensor_config_t *config);


//void uart_init(void);
//bool ack_response(const uint8_t *data, int len);
//void uart_read_task(void *arg);
//void send_command_to_rd03(const char *cmd, size_t len);

#endif // SENSOR_UART_H


