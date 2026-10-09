/**
 * @file bridge.h
 * @brief Phone (BLE) <-> robot (UART) line bridge, with the Atom's own commands
 *
 * Lines from the phone go to the robot's Bluetooth console unchanged, except
 * the commands the Atom answers itself (AT+CAM?, AT+WIFI, AT+ATOM?). Everything
 * the robot sends goes to the phone.
 */

#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void bridge_init(void);

/** BLE write from the phone (NimBLE host task) */
void bridge_from_phone(const uint8_t *data, size_t len);

/** Phone connected or gone (NimBLE host task) */
void bridge_phone_connected(bool connected);

/** Setup console on the USB-C port (idf.py monitor): the Atom's own commands. Never returns. */
void bridge_usb_console(void);

#endif // BRIDGE_H
