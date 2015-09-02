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


//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define NODEID           1
#define NETWORKID        100
#define FREQUENCY        RF69_868MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY       "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define LED              9
#define FLASH_CS         8
#define ACK_TIME         100
#define REPORT_EVERY     3
#define SERIAL_BAUD      115200
#define SERIAL_EN        //comment out if you don't want any serial verbose output
#define COMMAND_LIST_MAX 20
//*****************************************************************************************************************************

static byte reportCount = REPORT_EVERY;

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
} Payload;
Payload rData, lData;

typedef struct {
  int nodeid;
  byte cmd; /*1 - measure (S), 10 - update (C), 255 - reboot (C), */
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

int readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return (int)result;
}

static int smoothedAverage(int prev, int next, bool firstTime = 0)
{
  if (firstTime) return next;
  return (int)((prev + next) / 2);
}

void report_value(uint8_t senderid, String value)
{
  Serial.print("[");Serial.print(senderid);Serial.print("]");
  Serial.print("[");Serial.print(value);Serial.println("]");
}

String int2float_metric(String sensor_type, int data, float divisor, int prec)
{
  char tmp[10];
  dtostrf(data / divisor, 4, prec, tmp);

  sensor_type += ":";
  sensor_type += tmp;
  return sensor_type;
}
String byte2int_metric(String sensor_type, byte data)
{
  sensor_type += ":";
  sensor_type += data;
  return sensor_type;
}
String bool2int_metric(String sensor_type, bool data)
{
  sensor_type += ":";
  sensor_type += data;
  return sensor_type;
}
String int162int_metric(String sensor_type, int16_t data)
{
  sensor_type += ":";
  sensor_type += data;
  return sensor_type;
}

void doMeasure()
{
  DEBUGln("doMeasure()");
  bool firstTime = 0;
  if (lData.uptime == 0)
  {
    firstTime = 1;
  }

  lData.uptime++;
  if (lData.uptime == 255) lData.uptime = 50;

  lData.vcc = smoothedAverage(lData.vcc, readVcc(), firstTime);

}

void doReport()
{
  DEBUGln("doReport()");

  report_value(NODEID, int2float_metric("voltage", lData.vcc, 1000.0, 2));
  report_value(NODEID, byte2int_metric("uptime", lData.uptime));
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
            doMeasure();
            if (++reportCount >= REPORT_EVERY)
            {
              reportCount = 0;
              doReport();
            }
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
      if (rData.vcc) report_value(senderid, int2float_metric("voltage", rData.vcc, 1000.0, 2));
      if (rData.temp != 85) report_value(senderid, int2float_metric("temperature", rData.temp, 100.0, 1));
      if (rData.uptime) report_value(senderid, byte2int_metric("uptime", rData.uptime));
      if (rData.motionDetected) report_value(senderid, bool2int_metric("motion", rData.motionDetected));
      if (rData.failedReport) report_value(senderid, byte2int_metric("failedreport", rData.failedReport));
      if (rData.rssi) report_value(senderid, int162int_metric("rssi", rData.rssi));
      rData.processed = true;
    }
  }
}  
