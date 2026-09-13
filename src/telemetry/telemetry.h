/**
 * @file telemetry.h
 * @brief Control loop logging over the USB console
 *
 * The control task hands over a small record without blocking; a lower
 * priority task formats it and waits for UART space. Every record carries a
 * sequence number and the running drop count, so a host can prove a log is
 * complete.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
    uint32_t tick_ms;   /**< Time the sample was processed */
    float acc_deg;      /**< Raw accelerometer angle */
    float kalman;       /**< Kalman filter output */
    float comp;         /**< Complementary filter output */
    float tilt;         /**< Tilt used by the controller (0 = upright) */
    float p;            /**< PID terms and clamped output (0 when not balancing) */
    float i;
    float d;
    float out;
} Telemetry_Record_t;

/** Create the record queue; call before the scheduler starts */
void telemetry_init(void);

/**
 * @brief Hand a record to the logger without blocking
 *
 * Call from one task only (the control task).
 *
 * @return false if the queue was full and the record was dropped
 */
bool telemetry_submit(const Telemetry_Record_t *record);

/** Formats records and writes them to UART_PORT_USB */
void telemetry_task(void *args);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TELEMETRY_H
