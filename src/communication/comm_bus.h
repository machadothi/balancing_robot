#ifndef COMM_BUS_H_
#define COMM_BUS_H_

#include <stdint.h>

typedef struct {
    void (*init)();
    void (*write_byte)(uint8_t data);
    uint8_t (*read_byte)();
    void (*start)();
    void (*stop)();
} CommBus;

void comm_bus_init(CommBus *bus);
void comm_bus_write_byte(CommBus *bus, uint8_t data);
uint8_t comm_bus_read_byte(CommBus *bus);
void comm_bus_start(CommBus *bus);
void comm_bus_stop(CommBus *bus);

#endif // COMM_BUS_H_