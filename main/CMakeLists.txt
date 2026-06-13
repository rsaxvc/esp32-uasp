idf_component_register(
    SRCS "main.c" "uasp.c"
    INCLUDE_DIRS "."
    REQUIRES wear_levelling esp_partition
)
