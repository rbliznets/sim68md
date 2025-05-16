idf_component_register(SRCS "SIM68MD.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES task dataformat esp_driver_gpio esp_driver_uart esp_pm libnmea)
