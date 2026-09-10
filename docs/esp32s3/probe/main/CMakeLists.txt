idf_component_register(SRCS "main.c" INCLUDE_DIRS "." "${S3_PRIVATE_DIR}"
    REQUIRES esp_wifi esp_event esp_netif nvs_flash lwip esp_timer)
if(S3_HAL_ARCHIVE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE S3_REVIEWED_HAL=1)
endif()
