/**
 * @file camera_http.c
 * @brief GC0308 camera, Wi-Fi station and the MJPEG HTTP server
 *
 * The GC0308 has no JPEG encoder: frames are RGB565 and are compressed in
 * software (frame2jpg), about 10-15 frames/s at 320x240.
 */

#include "camera_http.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "nvs.h"

static const char *TAG = "cam";

/* AtomS3R-CAM (docs.m5stack.com/en/core/AtomS3R-CAM) */
#define CAM_POWER_GPIO      18          /* low = camera powered */
#define CAM_PIN_XCLK        21
#define CAM_PIN_SDA         12
#define CAM_PIN_SCL         9
#define CAM_PIN_D7          13
#define CAM_PIN_D6          11
#define CAM_PIN_D5          17
#define CAM_PIN_D4          4
#define CAM_PIN_D3          48
#define CAM_PIN_D2          46
#define CAM_PIN_D1          42
#define CAM_PIN_D0          3
#define CAM_PIN_VSYNC       10
#define CAM_PIN_HREF        14
#define CAM_PIN_PCLK        40

#define HOSTNAME            "balancebot"
#define NVS_NAMESPACE       "wifi"
#define RETRY_DELAY_MS      5000
#define SCAN_MAX_APS        20

#define PART_BOUNDARY       "frame"

typedef enum { WIFI_OFF, WIFI_CONNECTING, WIFI_CONNECTED } Wifi_State_t;

static SemaphoreHandle_t lock;
static esp_netif_t *netif;
static bool camera_ok;
static bool wifi_started;
static volatile bool scanning;      /* a scan drops the connection attempt: no retry meanwhile */
static Wifi_State_t wifi_state = WIFI_OFF;
static char wifi_ssid[33];
static char wifi_ip[16];

/* ==========================================================================
 * Camera
 * ========================================================================== */

static void camera_init(void) {
    gpio_set_direction(CAM_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_POWER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(500));     /* the sensor needs time after power-on */

    camera_config_t config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SDA,
        .pin_sccb_scl = CAM_PIN_SCL,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&config);
    camera_ok = (err == ESP_OK);
    if (camera_ok) {
        ESP_LOGI(TAG, "camera ready, sensor PID 0x%x", esp_camera_sensor_get()->id.PID);
    } else {
        ESP_LOGE(TAG, "camera init failed: %s (the BLE link still works)", esp_err_to_name(err));
    }
}

/* ==========================================================================
 * HTTP: / (page), /stream (MJPEG), /capture (one JPEG)
 * ========================================================================== */

static esp_err_t page_handler(httpd_req_t *req) {
    static const char page[] =
        "<!doctype html><meta name=viewport content='width=device-width'>"
        "<title>balancing robot</title><body style='margin:0;background:#111'>"
        "<img src='/stream' style='width:100%'>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

/** One frame as JPEG; the caller frees *jpg */
static bool grab_jpeg(uint8_t **jpg, size_t *len) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return false;
    }
    bool ok = frame2jpg(fb, CONFIG_ATOM_JPEG_QUALITY, jpg, len);
    esp_camera_fb_return(fb);
    return ok;
}

static esp_err_t capture_handler(httpd_req_t *req) {
    uint8_t *jpg = NULL;
    size_t len = 0;
    if (!camera_ok || !grab_jpeg(&jpg, &len)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no camera frame");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    esp_err_t err = httpd_resp_send(req, (const char *)jpg, (ssize_t)len);
    free(jpg);
    return err;
}

static esp_err_t stream_handler(httpd_req_t *req) {
    if (!camera_ok) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no camera");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" PART_BOUNDARY);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    ESP_LOGI(TAG, "stream started");

    esp_err_t err = ESP_OK;
    uint32_t frames = 0;
    TickType_t since = xTaskGetTickCount();
    while (err == ESP_OK) {
        uint8_t *jpg = NULL;
        size_t len = 0;
        if (!grab_jpeg(&jpg, &len)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        char header[96];
        int n = snprintf(header, sizeof(header),
                         "--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                         (unsigned)len);
        err = httpd_resp_send_chunk(req, header, n);
        err = err ? err : httpd_resp_send_chunk(req, (const char *)jpg, (ssize_t)len);
        err = err ? err : httpd_resp_send_chunk(req, "\r\n", 2);
        free(jpg);

        if (++frames == 50) {
            uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount() - since);
            ESP_LOGI(TAG, "%.1f frames/s, %u bytes/frame", 50000.0f / (float)ms, (unsigned)len);
            frames = 0;
            since = xTaskGetTickCount();
        }
    }
    ESP_LOGI(TAG, "stream ended");
    return ESP_OK;
}

static void http_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start");
        return;
    }
    static const httpd_uri_t uris[] = {
        { .uri = "/", .method = HTTP_GET, .handler = page_handler },
        { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler },
        { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
}

/* ==========================================================================
 * Wi-Fi station
 * ========================================================================== */

static void set_state(Wifi_State_t state, const char *ip) {
    xSemaphoreTake(lock, portMAX_DELAY);
    wifi_state = state;
    snprintf(wifi_ip, sizeof(wifi_ip), "%s", ip);
    xSemaphoreGive(lock);
}

static void retry_task(void *param) {
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    if (wifi_state == WIFI_CONNECTING) {
        esp_wifi_connect();
    }
    vTaskDelete(NULL);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (wifi_state != WIFI_OFF) {
            esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_state != WIFI_OFF && !scanning) {
            const wifi_event_sta_disconnected_t *e = data;
            ESP_LOGW(TAG, "Wi-Fi \"%s\" lost or not found (reason %d), retrying", wifi_ssid, e->reason);
            set_state(WIFI_CONNECTING, "");
            xTaskCreate(retry_task, "wifi_retry", 2048, NULL, 2, NULL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&e->ip_info.ip));
        set_state(WIFI_CONNECTED, ip);
        ESP_LOGI(TAG, "Wi-Fi up: http://%s/stream", ip);
    }
}

static void wifi_connect(const char *ssid, const char *password) {
    wifi_config_t config = { 0 };
    snprintf((char *)config.sta.ssid, sizeof(config.sta.ssid), "%s", ssid);
    snprintf((char *)config.sta.password, sizeof(config.sta.password), "%s", password);
    config.sta.threshold.authmode = password[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;

    xSemaphoreTake(lock, portMAX_DELAY);
    snprintf(wifi_ssid, sizeof(wifi_ssid), "%s", ssid);
    wifi_state = WIFI_CONNECTING;
    wifi_ip[0] = '\0';
    xSemaphoreGive(lock);

    if (wifi_started) {
        esp_wifi_disconnect();
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
        esp_wifi_connect();
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
        ESP_ERROR_CHECK(esp_wifi_start());     /* STA_START connects */
        wifi_started = true;
    }
    ESP_LOGI(TAG, "joining Wi-Fi \"%s\"", ssid);
}

static void wifi_init(void) {
    netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, HOSTNAME);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));    /* our own NVS keys */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));

    char ssid[33] = CONFIG_ATOM_WIFI_SSID;
    char password[65] = CONFIG_ATOM_WIFI_PASSWORD;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t n = sizeof(ssid);
        if (nvs_get_str(nvs, "ssid", ssid, &n) == ESP_OK) {
            n = sizeof(password);
            if (nvs_get_str(nvs, "pass", password, &n) != ESP_OK) {
                password[0] = '\0';
            }
        }
        nvs_close(nvs);
    }

    if (ssid[0]) {
        wifi_connect(ssid, password);
    } else {
        ESP_LOGW(TAG, "no Wi-Fi network set: send AT+WIFI=<ssid>,<password> (app or USB console)");
    }
}

/* ==========================================================================
 * Public
 * ========================================================================== */

void camera_http_init(void) {
    lock = xSemaphoreCreateMutex();
    camera_init();
    wifi_init();
    http_start();
}

void camera_http_url(char *buf, size_t len) {
    xSemaphoreTake(lock, portMAX_DELAY);
    if (wifi_state == WIFI_CONNECTED && camera_ok) {
        snprintf(buf, len, "http://%s/stream", wifi_ip);
    } else {
        buf[0] = '\0';
    }
    xSemaphoreGive(lock);
}

void camera_http_wifi_status(char *buf, size_t len) {
    static const char *const names[] = { "off", "connecting", "connected" };
    xSemaphoreTake(lock, portMAX_DELAY);
    snprintf(buf, len, "%s,%s,%s", wifi_ssid, names[wifi_state], wifi_ip);
    xSemaphoreGive(lock);
}

bool camera_http_wifi_scan(char *buf, size_t len) {
    if (!wifi_started) {
        if (esp_wifi_start() != ESP_OK) {      /* no network set yet: start without joining */
            return false;
        }
        wifi_started = true;
    }
    /* The driver refuses to scan while it tries to join */
    scanning = true;
    if (wifi_state == WIFI_CONNECTING) {
        esp_wifi_disconnect();
    }
    wifi_scan_config_t config = { .show_hidden = false };
    wifi_ap_record_t *aps = calloc(SCAN_MAX_APS, sizeof(*aps));
    uint16_t count = SCAN_MAX_APS;
    esp_err_t err = aps ? esp_wifi_scan_start(&config, true) : ESP_ERR_NO_MEM;
    if (err == ESP_OK) {
        err = esp_wifi_scan_get_ap_records(&count, aps);
    }
    scanning = false;
    if (wifi_state == WIFI_CONNECTING) {
        esp_wifi_connect();
    }

    size_t pos = 0;
    buf[0] = '\0';
    for (uint16_t i = 0; err == ESP_OK && i < count; i++) {
        const char *ssid = (const char *)aps[i].ssid;
        bool skip = ssid[0] == '\0' || strchr(ssid, '\t') || strchr(ssid, ',');
        for (uint16_t j = 0; j < i && !skip; j++) {
            skip = strcmp(ssid, (const char *)aps[j].ssid) == 0;    /* strongest first */
        }
        if (skip) {
            continue;
        }
        int n = snprintf(buf + pos, len - pos, "%s%d %s", pos ? "\t" : "", aps[i].rssi, ssid);
        if (n < 0 || (size_t)n >= len - pos) {
            buf[pos] = '\0';
            break;
        }
        pos += (size_t)n;
    }
    free(aps);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
    }
    return err == ESP_OK;
}

bool camera_http_set_wifi(const char *ssid, const char *password) {
    if (ssid[0] == '\0' || strlen(ssid) > 32 || strlen(password) > 64 ||
        (password[0] && strlen(password) < 8)) {
        return false;
    }
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return false;
    }
    bool ok = nvs_set_str(nvs, "ssid", ssid) == ESP_OK &&
              nvs_set_str(nvs, "pass", password) == ESP_OK &&
              nvs_commit(nvs) == ESP_OK;
    nvs_close(nvs);
    if (ok) {
        wifi_connect(ssid, password);
    }
    return ok;
}
