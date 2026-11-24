idf_component_register(SRCS "pca9685_servo.c" "motor.c" "main.c" "microphone.c" "speaker.c" "wife.c" "voice.c" "cloud_ai.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip driver esp-sr esp_http_client json mbedtls helix_mp3)
