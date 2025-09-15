idf_component_register(SRCS "st7796s.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_ledc spiffs)

spiffs_create_partition_image(storage ${PROJECT_DIR}/spiffs_image FLASH_IN_PROJECT)