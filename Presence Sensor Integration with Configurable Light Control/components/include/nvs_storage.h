// nvs_storage.h
#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "config.h"

void nvs_save_config(const sensor_config_t *config);
void nvs_load_config(sensor_config_t *config);
void nvs_storage_init(void);

#endif // NVS_STORAGE_H
