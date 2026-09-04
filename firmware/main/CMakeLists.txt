idf_component_register(SRCS "sensores_microros_main.c" "as5600.c" "pote.c"
                       PRIV_REQUIRES esp_adc freertos esp_driver_gpio esp_driver_i2c esp_timer micro_ros_espidf_component
                       INCLUDE_DIRS ".")
