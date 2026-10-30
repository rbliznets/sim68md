idf_component_register(SRCS "nmea_parser.cpp" "SIM68MD.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES task dataformat esp_driver_gpio esp_driver_uart esp_pm esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Os)
