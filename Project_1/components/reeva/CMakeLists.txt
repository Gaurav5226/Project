set(COMPONENT_SRCS "Src/uart_handler.c" "Src/nvs_storage.c" "Src/pwm_control.c")
set(COMPONENT_ADD_INCLUDEDIRS "Include")

idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
                       PRIV_REQUIRES driver nvs_flash json)