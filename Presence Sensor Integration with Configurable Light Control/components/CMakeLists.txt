set(COMPONENT_SRCS "src/uart_pc.c" "src/sensor_uart.c" "src/nvs_storage.c" "src/pwm_control.c")
set(COMPONENT_ADD_INCLUDEDIRS "Include")

idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
                       PRIV_REQUIRES driver nvs_flash json esp_timer)