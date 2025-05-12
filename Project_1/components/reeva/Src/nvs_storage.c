#include "nvs_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_storage";
#define NVS_NAMESPACE "lightcfg"

// Intialise NVS 
esp_err_t nvs_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_LOGW(TAG, "NVS partition was crashed, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to init NVS: %s", esp_err_to_name(err));
    }

    return err;
}

// Write a new value in NVS
esp_err_t nvs_storage_write(const light_data_t *data)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err |= nvs_set_u16(handle, "dimming", data->dimming);
    err |= nvs_set_u16(handle, "tuning", data->tuning);

    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved dimming=%d, tuning=%d", data->dimming, data->tuning);
    else
        ESP_LOGE(TAG, "Failed to save settings: %s", esp_err_to_name(err));

    return err;
}

// Read data from NVS 
esp_err_t nvs_storage_read(light_data_t *data)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) 
    {
        ESP_LOGW(TAG, "No saved config found");
        return err;
    }

    err = nvs_get_u16(handle, "dimming", &data->dimming);
    if (err != ESP_OK) 
    {
        nvs_close(handle);
        return err;
    }

    err = nvs_get_u16(handle, "tuning", &data->tuning);
    if (err != ESP_OK) 
    {
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Read from NVS: dimming=%d, tuning=%d", data->dimming, data->tuning);
    return ESP_OK;
}
