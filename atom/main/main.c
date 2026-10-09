/**
 * @file main.c
 * @brief AtomS3R-CAM bridge for the balancing robot
 *
 * - BLE (Nordic UART Service) <-> the robot's Bluetooth console on the Grove
 *   port: the phone app's AT commands, replies and live data
 * - Wi-Fi: camera video at http://<ip>/stream, announced to the app by AT+CAM?
 *
 * The robot firmware is built with BT_MODULE=atom (115200 baud).
 */

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "ble_nus.h"
#include "bridge.h"
#include "camera_http.h"

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    bridge_init();
    ble_nus_init(CONFIG_ATOM_BLE_NAME, bridge_from_phone, bridge_phone_connected);
    camera_http_init();

    bridge_usb_console();
}
