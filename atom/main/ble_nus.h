/**
 * @file ble_nus.h
 * @brief BLE serial port: the Nordic UART Service (NUS), one phone at a time
 *
 * The phone writes to the RX characteristic and subscribes to TX notifications.
 * NUS is what most BLE terminal apps speak, so any of them can test the link.
 */

#ifndef BLE_NUS_H
#define BLE_NUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Bytes written by the phone; called from the NimBLE host task */
typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, size_t len);

/** A phone connected (true) or went away (false); called from the NimBLE host task */
typedef void (*ble_nus_conn_cb_t)(bool connected);

void ble_nus_init(const char *name, ble_nus_rx_cb_t on_rx, ble_nus_conn_cb_t on_conn);

/** Notify the phone, split to the negotiated MTU. False if nobody listens. */
bool ble_nus_send(const uint8_t *data, size_t len);

#endif // BLE_NUS_H
