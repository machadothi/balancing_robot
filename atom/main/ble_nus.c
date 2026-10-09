/**
 * @file ble_nus.c
 * @brief Nordic UART Service on NimBLE: advertising, one connection, notify
 */

#include "ble_nus.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble";

/* 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E, little-endian */
#define NUS_UUID(x) BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
                                     0x93, 0xf3, 0xa3, 0xb5, (x), 0x00, 0x40, 0x6e)

static const ble_uuid128_t nus_service = NUS_UUID(0x01);
static const ble_uuid128_t nus_rx = NUS_UUID(0x02);     /* phone -> robot */
static const ble_uuid128_t nus_tx = NUS_UUID(0x03);     /* robot -> phone */

#define SEND_RETRIES 50                 /* x 2 ms while the controller's buffers are full */

static const char *adv_name;
static ble_nus_rx_cb_t rx_cb;
static ble_nus_conn_cb_t conn_cb;
static uint8_t own_addr_type;
static uint16_t tx_handle;
static volatile uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool subscribed;

static void advertise(void);

static int access_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    uint8_t buf[256];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (rx_cb) {
        rx_cb(buf, len);
    }
    return 0;
}

static const struct ble_gatt_svc_def services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nus_rx.u,
                .access_cb = access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &nus_tx.u,
                .access_cb = access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_handle,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "phone connected");
            if (conn_cb) {
                conn_cb(true);
            }
        } else {
            advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "phone disconnected (reason 0x%x)", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        subscribed = false;
        if (conn_cb) {
            conn_cb(false);
        }
        advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == tx_handle) {
            subscribed = event->subscribe.cur_notify;
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %u", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;

    default:
        break;
    }
    return 0;
}

/* The service UUID in the advertisement, the name in the scan response: both
 * do not fit in 31 bytes together */
static void advertise(void) {
    struct ble_hs_adv_fields fields = { 0 };
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &nus_service;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);

    struct ble_hs_adv_fields rsp = { 0 };
    rsp.name = (const uint8_t *)adv_name;
    rsp.name_len = (uint8_t)strlen(adv_name);
    rsp.name_is_complete = 1;
    rc = rc ? rc : ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = rc ? rc : ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "advertising failed: %d", rc);
    }
}

static void on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &own_addr_type);
    advertise();
    ESP_LOGI(TAG, "advertising as \"%s\"", adv_name);
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "host reset: %d", reason);
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_nus_init(const char *name, ble_nus_rx_cb_t on_rx, ble_nus_conn_cb_t on_conn) {
    adv_name = name;
    rx_cb = on_rx;
    conn_cb = on_conn;

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(services));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(services));
    ble_svc_gap_device_name_set(name);

    nimble_port_freertos_init(host_task);
}

bool ble_nus_send(const uint8_t *data, size_t len) {
    uint16_t conn = conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || !subscribed) {
        return false;
    }
    size_t chunk = ble_att_mtu(conn) - 3u;
    if (chunk < 20u || chunk > 244u) {
        chunk = 20u;
    }

    while (len > 0) {
        size_t n = len < chunk ? len : chunk;
        int rc = BLE_HS_ENOMEM;
        for (int attempt = 0; attempt < SEND_RETRIES && rc == BLE_HS_ENOMEM; attempt++) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(data, (uint16_t)n);
            rc = om ? ble_gatts_notify_custom(conn, tx_handle, om) : BLE_HS_ENOMEM;   /* consumes om */
            if (rc == BLE_HS_ENOMEM) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        if (rc != 0) {
            return false;
        }
        data += n;
        len -= n;
    }
    return true;
}
