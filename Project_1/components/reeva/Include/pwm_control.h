#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "esp_err.h"
#include "light_control.h"
#include "uart_handler.h"

esp_err_t pwm_control_init(void);
void pwm_control_start(void);
esp_err_t light_control_queue_init(void);
esp_err_t send_to_light_queue(const light_data_t *data);

#endif
