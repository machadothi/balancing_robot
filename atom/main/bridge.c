/**
 * @file bridge.c
 * @brief Phone (BLE) <-> robot (UART) line bridge
 */

#include "bridge.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "ble_nus.h"
#include "camera_http.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

static const char *TAG = "bridge";

#define ATOM_VERSION        "1.0"
#define ROBOT_UART          UART_NUM_1
#define UART_BUF_SIZE       1024
#define LINE_LEN            160     /* longer lines from the phone are dropped */
#define REPLY_MAX           640     /* AT+WIFISCAN? lists up to 20 networks */

static StreamBufferHandle_t from_phone;
static volatile bool stop_pending;

/* ==========================================================================
 * The Atom's own commands, answered like the robot's (src/cmd/at_cmd.c):
 * "+NAME:value" for a query, "OK" or "ERROR:n" otherwise
 * ========================================================================== */

#define ERROR_FAILED        1       /* AT_ERROR */
#define ERROR_INVALID       3       /* AT_ERROR_INVALID_PARAM */

/** True if [line] is ours; [reply] then holds the answer without line ending */
static bool local_command(const char *line, char *reply, size_t len) {
    char text[96];

    if (strcasecmp(line, "AT+CAM?") == 0) {
        camera_http_url(text, sizeof(text));
        snprintf(reply, len, "+CAM:%s", text);
    } else if (strcasecmp(line, "AT+WIFI?") == 0) {
        camera_http_wifi_status(text, sizeof(text));
        snprintf(reply, len, "+WIFI:%s", text);
    } else if (strncasecmp(line, "AT+WIFI=", 8) == 0) {
        /* AT+WIFI=<ssid>,<password>: the SSID ends at the first comma */
        char args[LINE_LEN];
        snprintf(args, sizeof(args), "%s", line + 8);
        char *comma = strchr(args, ',');
        const char *password = "";
        if (comma) {
            *comma = '\0';
            password = comma + 1;
        }
        bool ok = camera_http_set_wifi(args, password);
        snprintf(reply, len, ok ? "OK" : "ERROR:%d", ERROR_INVALID);
    } else if (strcasecmp(line, "AT+WIFISCAN?") == 0) {
        int n = snprintf(reply, len, "+WIFISCAN:");
        if (!camera_http_wifi_scan(reply + n, len - (size_t)n)) {
            snprintf(reply, len, "ERROR:%d", ERROR_FAILED);
        }
    } else if (strcasecmp(line, "AT+ATOM?") == 0) {
        snprintf(reply, len, "+ATOM:" ATOM_VERSION);
    } else {
        return false;
    }
    return true;
}

/* ==========================================================================
 * Robot -> phone
 * ========================================================================== */

static void robot_rx_task(void *param) {
    uint8_t buf[128];
    for (;;) {
        int n = uart_read_bytes(ROBOT_UART, buf, sizeof(buf), pdMS_TO_TICKS(5));
        if (n > 0) {
            (void)ble_nus_send(buf, (size_t)n);     /* dropped while nobody listens */
        }
    }
}

/* ==========================================================================
 * Phone -> robot, one line at a time
 * ========================================================================== */

static void to_robot(const char *line) {
    uart_write_bytes(ROBOT_UART, line, strlen(line));
    uart_write_bytes(ROBOT_UART, "\r", 1);
}

/* The phone is gone: zero the drive targets now instead of waiting for the
 * robot's 1 s dead-man. Balancing goes on. */
static void stop_driving(void) {
    to_robot("AT+VELOCITY=0");
    vTaskDelay(pdMS_TO_TICKS(30));      /* the robot takes one line at a time */
    to_robot("AT+TURN=0");
    uart_wait_tx_done(ROBOT_UART, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(30));
    uart_flush_input(ROBOT_UART);       /* their OKs are nobody's */
}

static void phone_task(void *param) {
    char line[LINE_LEN];
    size_t pos = 0;
    bool overflow = false;

    for (;;) {
        uint8_t buf[64];
        size_t n = xStreamBufferReceive(from_phone, buf, sizeof(buf), pdMS_TO_TICKS(100));

        if (stop_pending) {
            stop_pending = false;
            pos = 0;
            stop_driving();
        }

        for (size_t i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c != '\r' && c != '\n') {
                if (pos < sizeof(line) - 1u) {
                    line[pos++] = c;
                } else {
                    overflow = true;
                }
                continue;
            }
            line[pos] = '\0';
            if (pos > 0 && !overflow) {
                char reply[REPLY_MAX + 2];
                if (local_command(line, reply, REPLY_MAX)) {
                    strcat(reply, "\r\n");
                    (void)ble_nus_send((const uint8_t *)reply, strlen(reply));
                } else {
                    to_robot(line);
                }
            }
            pos = 0;
            overflow = false;
        }
    }
}

/* ==========================================================================
 * Public
 * ========================================================================== */

void bridge_from_phone(const uint8_t *data, size_t len) {
    if (xStreamBufferSend(from_phone, data, len, 0) != len) {
        ESP_LOGW(TAG, "phone input dropped");
    }
}

void bridge_phone_connected(bool connected) {
    if (!connected) {
        stop_pending = true;
    }
}

void bridge_init(void) {
    const uart_config_t config = {
        .baud_rate = CONFIG_ATOM_ROBOT_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(ROBOT_UART, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ROBOT_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(ROBOT_UART, CONFIG_ATOM_ROBOT_TX_GPIO, CONFIG_ATOM_ROBOT_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* An unplugged robot leaves RX floating: keep it idle-high */
    gpio_pullup_en(CONFIG_ATOM_ROBOT_RX_GPIO);

    from_phone = xStreamBufferCreate(1024, 1);
    xTaskCreate(robot_rx_task, "robot_rx", 4096, NULL, 10, NULL);
    xTaskCreate(phone_task, "phone", 6144, NULL, 9, NULL);
    ESP_LOGI(TAG, "robot UART: TX GPIO%d, RX GPIO%d, %d baud",
             CONFIG_ATOM_ROBOT_TX_GPIO, CONFIG_ATOM_ROBOT_RX_GPIO, CONFIG_ATOM_ROBOT_BAUDRATE);
}

void bridge_usb_console(void) {
    usb_serial_jtag_driver_config_t usb = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    esp_vfs_usb_serial_jtag_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);

    char line[LINE_LEN];
    size_t pos = 0;
    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (c != '\r' && c != '\n') {
            if (pos < sizeof(line) - 1u) {
                line[pos++] = (char)c;
            }
            continue;
        }
        line[pos] = '\0';
        if (pos > 0) {
            char reply[REPLY_MAX];
            if (!local_command(line, reply, sizeof(reply))) {
                snprintf(reply, sizeof(reply), "ERROR:2 (here: AT+WIFISCAN?, AT+WIFI=<ssid>,<password>, AT+WIFI?, AT+CAM?, AT+ATOM?)");
            }
            printf("%s\n", reply);
        }
        pos = 0;
    }
}
