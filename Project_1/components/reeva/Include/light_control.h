#ifndef LIGHT_CONTROL_H
#define LIGHT_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

// light data structure
typedef struct 
{
    uint16_t dimming;
    uint16_t tuning;
} light_data_t;

esp_err_t light_control_queue_init(void);

#endif
