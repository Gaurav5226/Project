#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "esp_err.h"

esp_err_t uart_init(void);
void uart_start(void);
void uart_send_ack(uint16_t dimming, uint16_t tuning);

#endif
