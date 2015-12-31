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
#include "config.h"

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1); Serial.flush();}
  #define DEBUGln(input) {Serial.println(input); delay(1); Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
SPIFlash flash(FLASH_CS, 0xEF30); //EF40 for 16mbit windbond chip

typedef struct {
  int           vcc;
  int           temp = 85; //DS18B20 returns 85 in case of error
  boolean       motionDetected = false;
  boolean       processed = false;
  byte          failedReport = 0;
  byte          locked = 0;
  byte          uptime = 0;
  unsigned int  token = 0;
  int16_t       rssi = 0;
  unsigned short version = VERSION;
} Payload;
Payload rData, lData;

typedef struct {
  int nodeid;
  byte cmd; /*1 - measure (S), 10 - update (C), 255 - reboot (C,S), */
} remote_cmd;
LinkedList<remote_cmd> cmd_list;

typedef struct {
  unsigned int token = 0;
  byte         cmd = 0;
} ACKPayload;

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

  if (lData.uptime == 255) lData.uptime = 50;
  report_value(NODEID, value2metric("uptime", ++lData.uptime));
  report_value(NODEID, value2metric("version", lData.version));
}

void setup()
{
  Blink(LED,10);
  Serial.begin(SERIAL_BAUD);
  DEBUGln("Setup...");
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.encrypt(ENCRYPTKEY);

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

void loop()
{
  uint8_t senderid;
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
      if (targetId == NODEID)
      {
        switch (input.toInt())
        {
          case 1:
            doReport();
            break;
          case 255:
            wdt_enable(WDTO_15MS);
            while(1);
            break;
        }
      }
      else if (targetId != NODEID && targetId != RF69_BROADCAST_ADDR && targetId > 0 && targetId < RF69_BROADCAST_ADDR)
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
  
  if (radio.receiveDone())
  {
    if (radio.DATALEN > 0 && radio.DATALEN == sizeof(Payload))
    {
      rData = *(Payload*) radio.DATA;
      senderid = radio.SENDERID;
      
      if (radio.ACKRequested())
      {
        ACKPayload ackData;
        if (rData.token) ackData.token = rData.token;
      
        for (int i = 0; i < cmd_list.size(); i++)
        {
          if (cmd_list.get(i).nodeid == senderid)
          {
            ackData.cmd = cmd_list.get(i).cmd;
            cmd_list.remove(i);
            break;
          }
        }
        radio.sendACK((const void*)&ackData, sizeof(ackData));
        if (ackData.cmd == 10) CheckForSerialHEX((byte *)"FLX?", 4, radio, senderid, ACK_TIME*30, ACK_TIME, true);
      }
    }
    
    Blink(LED,3);

    if (!rData.processed)
    {
      if (rData.vcc) report_value(senderid, value2metric("voltage", rData.vcc, 1000.0, 2));
      if (rData.temp != 85) report_value(senderid, value2metric("temperature", rData.temp, 100.0, 1));
      if (rData.uptime) report_value(senderid, value2metric("uptime", rData.uptime));
      if (rData.motionDetected) report_value(senderid, value2metric("motion", rData.motionDetected));
      if (rData.failedReport) report_value(senderid, value2metric("failedreport", rData.failedReport));
      if (rData.rssi) report_value(senderid, value2metric("rssi", rData.rssi));
      if (rData.version) report_value(senderid, value2metric("version", rData.version));
      rData.processed = true;
    }
  }
}  
