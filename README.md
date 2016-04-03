# moteino

Sketches for moteino compatible with https://github.com/bkupidura/mgw/

## Hardware

* Moteino boards (+FTDI adapter to upload board code).
 - Gateway w/ RFM69HW radio+flash
 - Nodes w/ RFM69W radio+flash
* Rassberry PI (or any other arduino compatible computer).

## Instalation on RassberryPI

```
$ git clone https://github.com/bkupidura/moteino.git
$ cd moteino
$ git clone https://github.com/sudar/Arduino-Makefile.git
$ sudo apt-get install arduino-core
$ sudo ln -s ~/Arduino/* /usr/share/arduino/libraries/
```

Modify Makefile in gateway and node directory.
If you encounter strange compilation errors, try to remove '-lc' flag from Arduino.mk:

> sed -i'' -e 's/-lc //' Arduino-mk/Arduino.mk

## Gateway

### Compilation

```
$ cd gateway
$ make raw_upload
```

### Commands

Gateway will accept on serial commands in format:

NODE_ID:CMD

10:255

#### Command list

* 1 - do measure (supported on gateway)
* 10 - handle wireless programming (supported on node)
* 11 - LED blink (supported on node)
* 255 - do reset (supported on gateway/node)

## Node

### Compilation

#### Local

```
$ cd node
$ make NODEID=10 serial_upload
```

#### Wireless

Disable mgw-gateway process if used. (https://github.com/bkupidura/mgw/)

```
$ cd node
$ make NODEID=10 wireless_upload
```

Enable mgw-gateway process if used.

## Payload L3

L3 is used to authenticate packet by sender and recipient.
Inside L3 packet, there is L4 with actual data.

* token - token used to verify ACK response by sender node, ACK should same token as packet
* id - id used to verify packet by recipient node
* type - type of packet (ERROR, NULL, MEASURE, COMMAND)
* size - size of L4 (data) payload
* data - L4 payload

## Payload L4 measure

L4 measure payload contains metrics reported by board.

* vcc - current voltage
* temp - temperature if DALLAS available
* motionDetected - reed switch (interrupt)
* failedReport - count of failed reports, it will be reset after first successfull report to gateway
* uptime - increased in each doReport, when uptime == 255 it will be set to 50, uptime < 50 means board was rebooted
* rssi - rssi from radio
