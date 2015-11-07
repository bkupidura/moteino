BOARD_TAG          = Moteino
ARDUINO_LIBS       = RFM69 SPIFlash SPI WirelessHEX69 jeelib arduino-onewire PinChangeInt
BOARDS_TXT         = ../boards.txt
MONITOR_PORT       = /dev/cu.usbserial-DA01IIT3
MONITOR_BAUDRATE   = 115200
WP_SERIAL          = /dev/ttyUSB0
WP_BIN             = ../bin/WirelessProgramming.py

GIT_VERSION        = $(shell git rev-list HEAD --count)
CPPFLAGS           = -DVERSION=$(GIT_VERSION) -DNODEID=${NODEID}

include ../Arduino-mk/Arduino.mk

ifndef NODEID
    $(error NODEID is missing)
endif

wireless_programming:
	$(WP_BIN) -f $(TARGET_HEX) -t $(NODEID) -s $(WP_SERIAL)


wireless_upload: clean all $(TARGET_HEX) verify_size wireless_programming
serial_upload: clean all $(TARGET_HEX) verify_size $(MAKE) do_upload
