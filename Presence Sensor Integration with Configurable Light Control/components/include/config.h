// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

typedef struct 
{
    uint8_t range;
    uint16_t timeout;
    uint16_t on_level;
    uint16_t off_level;
} sensor_config_t;

extern sensor_config_t global_config;

#endif // CONFIG_H
