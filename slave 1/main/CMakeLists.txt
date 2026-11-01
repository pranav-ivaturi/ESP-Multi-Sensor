idf_component_register(
    SRCS "main.c" "max31865.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc
)