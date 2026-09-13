/* Host stand-in for libopencm3/cm3/scb.h: AT+RESET does nothing on the host */
#ifndef HOST_SCB_H
#define HOST_SCB_H

static inline void scb_reset_system(void) {}

#endif // HOST_SCB_H
