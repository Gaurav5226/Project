#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "esp_err.h"
#include "light_control.h"

esp_err_t nvs_storage_init(void);
esp_err_t nvs_storage_write(const light_data_t *data);
esp_err_t nvs_storage_read(light_data_t *data);

#endif
