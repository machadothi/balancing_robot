######################################################################
#  Project Makefile
######################################################################

BINARY		= main

SRCFILES := main.c
SRCFILES += rtos/heap_4.c
SRCFILES += rtos/list.c
SRCFILES += rtos/port.c
SRCFILES += rtos/queue.c
SRCFILES += rtos/tasks.c
SRCFILES += rtos/opencm3.c
SRCFILES += communication/i2c.c
SRCFILES += communication/uart.c
SRCFILES += imu/imu.c
SRCFILES += imu/mpu6050.c
SRCFILES += log/log.c
SRCFILES += motor/motor.c
SRCFILES += robot/robot.c

CFLAGS += -I./
LDLIBS		+= -lm

LDSCRIPT	= stm32f103c8t6.ld

# DEPS		= 	# Any additional dependencies for your build
# CLOBBER	+= 	# Any additional files to be removed with "make clobber"

include ../../../Makefile.incl
include ../../Makefile.rtos

all: elf bin

######################################################################
#  NOTES:
#	1. remove any modules you don't need from SRCFILES
#	2. "make clean" will remove *.o etc., but leaves *.elf, *.bin
#	3. "make clobber" will "clean" and remove *.elf, *.bin etc.
#	4. "make flash" will perform:
#	   st-flash write main.bin 0x8000000
######################################################################
