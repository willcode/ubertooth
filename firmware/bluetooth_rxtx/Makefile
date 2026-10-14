# Hey Emacs, this is a -*- makefile -*-

# Target file name (without extension).
TARGET = bluetooth_rxtx

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c \
	bluetooth.c \
	bluetooth_le.c \
	ubertooth_usb.c \
	ubertooth_rssi.c \
	ubertooth_cs.c \
	ubertooth_clock.c \
	ubertooth_dma.c \
	le_phy.c \
	queue.c \
	cc2400_rangetest.c \
	ego.c \
	debug_uart.c \
	tinyprintf.c \
	fault.c \
	xmas.c \
	$(LIBS_PATH)/usb_serial.c \
	$(LIBS_PATH)/serial_fifo.c \
	$(LIBS_PATH)/LPC17xx_Startup.c \
	$(LIBS_PATH)/LPC17xx_Interrupts.c \
	$(LIBS_PATH)/ubertooth.c \
	$(LPCUSB_PATH)/usbcontrol.c \
	$(LPCUSB_PATH)/usbinit.c \
	$(LPCUSB_PATH)/usbhw_lpc.c \
	$(LPCUSB_PATH)/usbstdreq.c

# Reset_Handler (LPC17xx_Startup.c) does all the C runtime setup.  Without
# this, newlib's crt0 is linked in and drags ~1 kB of stdio/atexit state
# (__sf, _impure_data, __atexit0) into the 16 kB local SRAM.
LINKER_OPTS += -nostartfiles

# fail the link if less than 4 kB of local SRAM is left for the stack
LINKER_OPTS += -Wl,--defsym=_min_stack_size=4096

include ../common.mk
