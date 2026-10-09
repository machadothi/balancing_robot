/**
 * @file camera_http.h
 * @brief Camera video over Wi-Fi: MJPEG at http://<ip>/stream
 *
 * Wi-Fi joins one network (station mode). The credentials come from AT+WIFI=
 * (kept in NVS) or, until then, from menuconfig. Without them Wi-Fi stays off
 * and only the BLE bridge runs.
 */

#ifndef CAMERA_HTTP_H
#define CAMERA_HTTP_H

#include <stdbool.h>
#include <stddef.h>

void camera_http_init(void);

/** "http://<ip>/stream" while Wi-Fi is up, else "" */
void camera_http_url(char *buf, size_t len);

/** "<ssid>,<off|connecting|connected>,<ip>" */
void camera_http_wifi_status(char *buf, size_t len);

/**
 * Networks in range, strongest first: "<rssi> <ssid>" separated by tabs.
 * Takes about 3 s. SSIDs with a comma or tab are left out (AT+WIFI= could not take them).
 */
bool camera_http_wifi_scan(char *buf, size_t len);

/** Store new credentials and reconnect. False if they are invalid. */
bool camera_http_set_wifi(const char *ssid, const char *password);

#endif // CAMERA_HTTP_H
