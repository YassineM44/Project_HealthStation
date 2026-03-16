idf_component_register(
    SRCS
        "../weight.c"
        "../mlx906.c"
        "../max30102.c"
        "../ad8232.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_rom esp_adc
)
