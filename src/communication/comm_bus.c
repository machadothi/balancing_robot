#include "comm_bus.h"

void comm_bus_init(CommBus *bus) {
    bus->init();
}

void comm_bus_write_byte(CommBus *bus, uint8_t data) {
    bus->write_byte(data);
}

uint8_t comm_bus_read_byte(CommBus *bus) {
    return bus->read_byte();
}

void comm_bus_start(CommBus *bus) {
    bus->start();
}

void comm_bus_stop(CommBus *bus) {
    bus->stop();
}