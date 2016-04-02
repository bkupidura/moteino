// **********************************************************************************************************
// GarageMote garage door controller base receiver sketch that works with Moteinos equipped with HopeRF RFM69W/RFM69HW
// Can be adapted to use Moteinos using RFM12B
// This is the sketch for the base, not the controller itself, and meant as another example on how to use a
// Moteino as a gateway/base/receiver
// 2014-07-14 (C) felix@lowpowerlab.com, http://www.LowPowerLab.com
// **********************************************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code/library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/4.0/
// **********************************************************************************

#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Ports.h>         //get it here: https://github.com/jcw/jeelib
#include <LinkedList.h>    //get it here: https://github.com/ivanseidel/LinkedList
#include <avr/wdt.h>
#include "../config.h"
#include "../lib/utils.h"

RFM69 radio;
SPIFlash flash(FLASH_CS, 0xEF30); //EF40 for 16mbit windbond chip

PayloadL4measure lDataL4measure;
LinkedList<remote_cmd> cmd_list;
unsigned int ids[ID_MAX] = {0};

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void report_value(uint8_t senderid, String value)
{
  Serial.print("[");Serial.print(senderid);Serial.print("]");
  Serial.print("[");Serial.print(value);Serial.println("]");
}

template<typename type> String value2metric(String sensor_type, type value)
{
    sensor_type += ":";
    sensor_type += value;
    return sensor_type;
}
String value2metric(String sensor_type, int value, float divisor, int prec)
{
  char tmp[10];
  dtostrf(value / divisor, 4, prec, tmp);

  sensor_type += ":";
  sensor_type += tmp;
  return sensor_type;
}

void doReport()
{
  DEBUGln("doReport()");

  if (lDataL4measure.uptime == 255) lDataL4measure.uptime = 50;
  report_value(GATEWAYID, value2metric("uptime", ++lDataL4measure.uptime));
}

void setup()
{
  Blink(LED,10);
  Serial.begin(SERIAL_BAUD);
  DEBUGln("Setup...");
  radio.initialize(FREQUENCY,GATEWAYID,NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.encrypt(ENCRYPTKEY);

  randomSeed(analogRead(0));

  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
  {
    DEBUGln("SPI Flash Init FAIL! (is chip present?)");
  }

  DEBUG("Network ID:");DEBUGln(NETWORKID);
  DEBUG("Encrypt key:");DEBUGln(ENCRYPTKEY);
  delay(1000);

}

void handle_serial_input()
{
  String input;
  if (Serial.available())
  {
    delay(10); //Wait for message
    while (Serial.available())
    {
      input += (char)Serial.read();
    }
    if (input.length() > 0)
    {
      byte targetId = input.toInt();
      byte colonIndex = input.indexOf(":");
      input.toUpperCase();
      DEBUG("Got input on serial: "); DEBUGln(input);
      if (targetId > 0) input = input.substring(colonIndex+1);
      if (targetId == GATEWAYID)
      {
        switch (input.toInt())
        {
          case CMD_MEASURE:
            doReport();
            break;
          case CMD_REBOOT:
            wdt_enable(WDTO_15MS);
            while(1);
            break;
        }
      }
      else if (targetId != GATEWAYID && targetId != RF69_BROADCAST_ADDR && targetId > 0 && targetId < RF69_BROADCAST_ADDR)
      {
        if (cmd_list.size() < COMMAND_LIST_MAX)
        {
          DEBUGln("Adding command for remote node");
          remote_cmd cmd = {targetId, (byte)input.toInt()};
          cmd_list.add(cmd);
        }
        else DEBUGln("Command list full");
      }
    }
  }
}

PayloadL3 build_ack(uint8_t senderid, unsigned int token, byte type)
{
  PayloadL3 ackDataL3;
  PayloadL4cmd ackDataL4cmd;

  ackDataL3.id = ids[senderid];

  if (token) ackDataL3.token = token;
  if (type == MSG_ERROR) ackDataL3.type = MSG_ERROR;
  else {
    for (int i = 0; i < cmd_list.size(); i++)
      if (cmd_list.get(i).nodeid == senderid)
      {
        ackDataL4cmd.cmd = cmd_list.get(i).cmd;
        ackDataL3.type = MSG_COMMAND;
        cmd_list.remove(i);
        break;
      }
    memcpy(&ackDataL3.data, &ackDataL4cmd, sizeof(ackDataL4cmd));
    ackDataL3.size = sizeof(ackDataL4cmd);
  }

  return ackDataL3;
}

PayloadL3ws handle_radio_input()
{
  PayloadL3ws rDataL3;
  PayloadL3 ackDataL3;
  PayloadL4cmd ackDataL4cmd;

  if (radio.receiveDone())
  {
    if (radio.DATALEN > 0)
    {
      rDataL3.payload = *(PayloadL3*) radio.DATA;
      rDataL3.senderid = radio.SENDERID;

      if (!check_id(rDataL3.payload.id, &ids[rDataL3.senderid]))
        rDataL3.payload.type = MSG_ERROR;

      if (radio.ACKRequested())
      {
        ackDataL3 = build_ack(rDataL3.senderid, rDataL3.payload.token, rDataL3.payload.type);
        radio.sendACK((const void*)&ackDataL3, sizeof(ackDataL3));

        if (ackDataL3.type == MSG_COMMAND){
          memcpy(&ackDataL4cmd, &ackDataL3.data, ackDataL3.size);
          if (ackDataL4cmd.cmd == CMD_UPDATE) CheckForSerialHEX((byte *)"FLX?", 4, radio, rDataL3.senderid, ACK_TIME*30, ACK_TIME, true);
        }
      }
    }
    Blink(LED,3);
  }
  return rDataL3;
}

void loop()
{
  PayloadL3ws rDataL3;
  PayloadL4measure rDataL4measure;

  handle_serial_input();
  rDataL3 = handle_radio_input();

  if (rDataL3.payload.type == MSG_MEASURE)
  {
    memcpy(&rDataL4measure, &rDataL3.payload.data, rDataL3.payload.size);
    if (rDataL4measure.vcc) report_value(rDataL3.senderid, value2metric("voltage", rDataL4measure.vcc, 1000.0, 2));
    if (rDataL4measure.temp != 85) report_value(rDataL3.senderid, value2metric("temperature", rDataL4measure.temp, 100.0, 1));
    if (rDataL4measure.uptime) report_value(rDataL3.senderid, value2metric("uptime", rDataL4measure.uptime));
    if (rDataL4measure.motionDetected) report_value(rDataL3.senderid, value2metric("motion", rDataL4measure.motionDetected));
    if (rDataL4measure.failedReport) report_value(rDataL3.senderid, value2metric("failedreport", rDataL4measure.failedReport));
    if (rDataL4measure.rssi) report_value(rDataL3.senderid, value2metric("rssi", rDataL4measure.rssi));
    rDataL3.payload.type = MSG_NULL;
  }
}  
