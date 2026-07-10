idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt esp_driver_gptimer esp_driver_i2c esp_driver_uart esp_driver_gpio esp_adc esp_timer nvs_flash
                        esp_wifi
                        esp_netif
                        esp_event
                        ota_http)
