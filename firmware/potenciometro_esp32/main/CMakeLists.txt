idf_component_register(SRCS "potenciometro_microros_main.c"
                       PRIV_REQUIRES esp_adc freertos esp_driver_gpio esp_timer micro_ros_espidf_component
                       INCLUDE_DIRS ".")
