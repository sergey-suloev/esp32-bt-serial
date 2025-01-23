idf_component_register(SRCS "main.c"
                            "spp_task.c"
                    PRIV_REQUIRES bt nvs_flash vfs esp_driver_gpio esp_driver_uart
                    INCLUDE_DIRS ".")
