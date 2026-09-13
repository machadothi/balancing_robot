/**
 * @file telemetry.c
 * @brief Control loop logging over the USB console
 *
 * Owns its AT command (AT+STREAM): the parser and the control loop know
 * nothing about streaming.
 */

#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"
#include "app/module.h"
#include "cmd/at_cmd.h"
#include "drivers/uart.h"
#include "telemetry/telemetry.h"
#include "util/fmt.h"

#define TELEMETRY_LINE_SIZE     200

typedef struct {
    uint32_t seq;
    Telemetry_Record_t record;
} Telemetry_Item_t;

static QueueHandle_t telemetry_queue = NULL;

/** AT+STREAM state */
static volatile bool streaming = false;

/** Assigned to every submitted record, including dropped ones, so drops show as gaps */
static uint32_t next_seq = 0;

static volatile uint32_t dropped = 0;

/* ==========================================================================
 * AT+STREAM
 * ========================================================================== */

static AT_Result_t query_stream(char *value, size_t size) {
    snprintf(value, size, "%d", streaming ? 1 : 0);
    return AT_OK;
}

static AT_Result_t set_stream(const float *values) {
    streaming = (values[0] != 0.0f);
    return AT_OK;
}

static const AT_Command_Def_t telemetry_commands[] = {
    { .name = "STREAM", .query = query_stream, .set = set_stream,
      .params = 1, .integer = true, .min = 0.0f, .max = 1.0f,
      .help = AT_HELP("Telemetry on the USB console, 0 or 1") },
};

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void telemetry_init(void) {
    telemetry_queue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(Telemetry_Item_t));
    at_cmd_register(telemetry_commands, sizeof(telemetry_commands) / sizeof(telemetry_commands[0]));
}

bool telemetry_submit(const Telemetry_Record_t *record) {
    if (!streaming) {
        return false;
    }

    Telemetry_Item_t item = { .seq = next_seq++, .record = *record };

    if (xQueueSend(telemetry_queue, &item, 0) != pdPASS) {
        dropped++;
        return false;
    }
    return true;
}

void telemetry_task(void *args) {
    (void)args;
    Telemetry_Item_t item;
    char line[TELEMETRY_LINE_SIZE];
    char v[8][16];

    for (;;) {
        if (xQueueReceive(telemetry_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }

        const Telemetry_Record_t *r = &item.record;
        int len = snprintf(line, sizeof(line),
            "seq: %lu | t: %lu | acc_deg: %s | kalman: %s | comp: %s | tilt: %s"
            " | p: %s | i: %s | d: %s | out: %s | drops: %lu\r\n",
            (unsigned long)item.seq, (unsigned long)r->tick_ms,
            fmt_fixed(v[0], sizeof(v[0]), r->acc_deg, 2),
            fmt_fixed(v[1], sizeof(v[1]), r->kalman, 2),
            fmt_fixed(v[2], sizeof(v[2]), r->comp, 2),
            fmt_fixed(v[3], sizeof(v[3]), r->tilt, 2),
            fmt_fixed(v[4], sizeof(v[4]), r->p, 2),
            fmt_fixed(v[5], sizeof(v[5]), r->i, 2),
            fmt_fixed(v[6], sizeof(v[6]), r->d, 2),
            fmt_fixed(v[7], sizeof(v[7]), r->out, 2),
            (unsigned long)dropped);

        if (len > 0 && (size_t)len < sizeof(line)) {
            /* Wait for UART space instead of dropping: only this task blocks,
             * and the control task never waits on it */
            (void)uart_write(UART_PORT_USB, line, (size_t)len, portMAX_DELAY);
        }
    }
}

const App_Module_t telemetry_module = {
    .name = "TELEM",
    .init = telemetry_init,
    .task = telemetry_task,
    .stack = 256,               /* line formatting */
    .priority = APP_PRIORITY_IO,
};
