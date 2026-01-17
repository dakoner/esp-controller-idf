idf_component_register(SRCS "console_example_main.c"
                       PRIV_REQUIRES esp_driver_ledc esp_wifi nvs_flash esp_ringbuf esp_timer fatfs 
                       INCLUDE_DIRS ".")
