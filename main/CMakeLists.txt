idf_component_register(SRCS "main.c" "microphone.c" "speaker.c" "cloud_ai.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_netif esp_event esp_http_client esp-tls json nvs_flash driver mbedtls)
