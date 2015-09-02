BOARD_TAG          = Moteino
ARDUINO_DIR        = /Applications/Arduino.app/Contents/Java/
ARDUINO_LIBS       = RFM69 SPIFlash SPI WirelessHEX69 jeelib arduino-onewire
BOARDS_TXT         = ../boards.txt
MONITOR_PORT       = /dev/cu.usbserial-DA01IIT3
MONITOR_BAUDRATE   = 115200
WP_SERIAL          = /dev/cu.usbserial-DA01IIY6
WP_BIN             = ../bin/WirelessProgramming.py
include ../Arduino-mk/Arduino.mk

ifndef NODEID
    $(error NODEID is missing)
endif

fix_nodeid:
	sed -i '' -e 's/^\(#define NODEID\).*/\1 $(NODEID)/' node.ino

wireless_programming:
	$(WP_BIN) -f $(TARGET_HEX) -t $(NODEID) -s $(WP_SERIAL)


wireless_upload: clean fix_nodeid all $(TARGET_HEX) verify_size wireless_programming
serial_upload: clean fix_nodeid all $(TARGET_HEX) verify_size $(MAKE) do_upload
