idf_component_register(SRCS "pca9685_servo.c" "motor.c" "main.c" "microphone.c" "speaker.c" "cloud_ai.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_netif esp_event esp_http_client esp-tls json nvs_flash driver mbedtls helix_mp3)

# 故意保留未使用的角度表/函数：只警告，不报错（不拦截编译）
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-const-variable
    -Wno-error=unused-variable
    -Wno-error=unused-function
)

