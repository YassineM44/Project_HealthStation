idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES AD8232 HX711 MAX30102 infraredtemp IR-sensor esp_timer)
