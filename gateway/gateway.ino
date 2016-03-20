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
#include "../lib/config.h"
#include "../lib/utils.h"

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1); Serial.flush();}
  #define DEBUGln(input) {Serial.println(input); delay(1); Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
SPIFlash flash(FLASH_CS, 0xEF30); //EF40 for 16mbit windbond chip

PayloadL3 rDataL3;
PayloadL4measure lDataL4measure;
LinkedList<remote_cmd> cmd_list;
byte ids[255] = {0};

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

PayloadL3 build_ack(uint8_t senderid, unsigned int token, bool processed)
{
  PayloadL3 ackDataL3;
  PayloadL4cmd ackDataL4cmd;

  ackDataL3.id = ids[senderid];

  if (token) ackDataL3.token = token;
  if (processed) ackDataL3.type = MSG_ERROR;
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

uint8_t handle_radio_input()
{
  uint8_t senderid;
  PayloadL3 ackDataL3;
  PayloadL4cmd ackDataL4cmd;

  if (radio.receiveDone())
  {
    if (radio.DATALEN > 0)
    {
      rDataL3 = *(PayloadL3*) radio.DATA;
      senderid = radio.SENDERID;

      if (rDataL3.id != ids[senderid])
        rDataL3.processed = true;
      else
        ids[senderid] = (byte) random(0, 256);

      if (radio.ACKRequested())
      {
        ackDataL3 = build_ack(senderid, rDataL3.token, rDataL3.processed);
        radio.sendACK((const void*)&ackDataL3, sizeof(ackDataL3));

        if (ackDataL3.type == MSG_COMMAND){
          memcpy(&ackDataL4cmd, &ackDataL3.data, ackDataL3.size);
          if (ackDataL4cmd.cmd == CMD_UPDATE) CheckForSerialHEX((byte *)"FLX?", 4, radio, senderid, ACK_TIME*30, ACK_TIME, true);
        }
      }
    }
    Blink(LED,3);
  }
  return senderid;
}

void loop()
{
  PayloadL4measure rDataL4measure;
  handle_serial_input();
  uint8_t senderid = handle_radio_input();

  if (!rDataL3.processed && rDataL3.type == MSG_MEASURE)
  {
    memcpy(&rDataL4measure, &rDataL3.data, rDataL3.size);
    if (rDataL4measure.vcc) report_value(senderid, value2metric("voltage", rDataL4measure.vcc, 1000.0, 2));
    if (rDataL4measure.temp != 85) report_value(senderid, value2metric("temperature", rDataL4measure.temp, 100.0, 1));
    if (rDataL4measure.uptime) report_value(senderid, value2metric("uptime", rDataL4measure.uptime));
    if (rDataL4measure.motionDetected) report_value(senderid, value2metric("motion", rDataL4measure.motionDetected));
    if (rDataL4measure.failedReport) report_value(senderid, value2metric("failedreport", rDataL4measure.failedReport));
    if (rDataL4measure.rssi) report_value(senderid, value2metric("rssi", rDataL4measure.rssi));
    rDataL3.processed = true;
  }
}  
