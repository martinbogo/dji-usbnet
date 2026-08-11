# dji-usbnet - userspace RNDIS -> utun bridge (HoRNDIS kext replacement)

CC      ?= cc
LIBUSB_CFLAGS := $(shell pkg-config --cflags libusb-1.0)
LIBUSB_LIBS   := $(shell pkg-config --libs libusb-1.0)

CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -std=c11
CFLAGS  += $(LIBUSB_CFLAGS)
LDFLAGS += $(LIBUSB_LIBS)

SRC := src/main.c src/usb.c src/utun.c src/bridge.c
OBJ := $(SRC:.c=.o)
BIN := dji-usbnet
PROBE := dji-probe

.PHONY: all clean run probe

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDFLAGS)

# Standalone USB descriptor dumper for identifying a device.
probe: $(PROBE)
$(PROBE): src/probe.c
	$(CC) $(CFLAGS) -o $@ src/probe.c $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# utun creation + ifconfig/route need root.
run: $(BIN)
	sudo ./$(BIN)

clean:
	rm -f $(OBJ) $(BIN) $(PROBE)
