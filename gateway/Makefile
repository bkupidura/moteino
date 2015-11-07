BOARD_TAG          = Moteino
ARDUINO_LIBS       = RFM69 SPIFlash SPI WirelessHEX69 jeelib LinkedList
BOARDS_TXT         = ../boards.txt
MONITOR_PORT       = /dev/ttyUSB0
MONITOR_BAUDRATE   = 115200

GIT_VERSION        = $(shell git rev-list HEAD --count)
CPPFLAGS           = "-DVERSION=$(GIT_VERSION)"
include ../Arduino-mk/Arduino.mk
