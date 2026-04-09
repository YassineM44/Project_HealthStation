idf_component_register(
    SRCS "ad8232.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc esp_timer
)
