// pwm_control.h
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

esp_err_t pwm_control_init(void);
void pwm_control_set_levels(int on, int off);
QueueHandle_t pwm_control_get_presence_queue(void);

#endif // PWM_CONTROL_H