idf_component_register(
    SRCS main.c bridge.c ble_nus.c camera_http.c
    INCLUDE_DIRS .
    REQUIRES driver nvs_flash esp_wifi esp_netif esp_event esp_http_server bt vfs
)
