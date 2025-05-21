#include "nvs_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sensor_uart.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "NVS";
sensor_config_t global_config;

void nvs_load_config(sensor_config_t *config)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        // Set default configuration
        config->range = 1;
        config->timeout = 3000;
        config->on_level = 90;
        config->off_level = 10;
        return;
    }

    size_t required_size = sizeof(sensor_config_t);
    err = nvs_get_blob(handle, "config_blob", config, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Configuration not found in NVS. Setting default values.");
        // Set default configuration
        config->range = 1;
        config->timeout = 3000;
        config->on_level = 90;
        config->off_level = 10;

        // Save default configuration to NVS
        err = nvs_set_blob(handle, "config_blob", config, sizeof(sensor_config_t));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write default config: %s", esp_err_to_name(err));
        }
        else
        {
            err = nvs_commit(handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to commit default config: %s", esp_err_to_name(err));
            }
        }
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to get config blob: %s", esp_err_to_name(err));
        // Set default configuration
        config->range = 1;
        config->timeout = 3000;
        config->on_level = 90;
        config->off_level = 10;
    }

    nvs_close(handle);
}


void nvs_save_config(const sensor_config_t *config)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, "config_blob", config, sizeof(sensor_config_t));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write config blob: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    nvs_load_config(&global_config);
    sensor_uart_config(&global_config);
}



void nvs_storage_init(void)
{
    ESP_LOGI(TAG, "Initializing NVS and loading config");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS flash init failed (%s), erasing...", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Ensure the 'storage' namespace exists
    nvs_handle_t handle;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        nvs_close(handle);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create 'storage' namespace: %s", esp_err_to_name(err));
    }
    nvs_load_config(&global_config);
}

