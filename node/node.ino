#include <RFM69.h> //get it here: http://github.com/lowpowerlab/rfm69
#include <SPI.h> //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: http://github.com/lowpowerlab/spiflash
#include <Ports.h> //get it here: https://github.com/jcw/jeelib
#include <OneWire.h>       //get it here: https://github.com/pbrook/arduino-onewire
#include <WirelessHEX69.h>
#include <avr/wdt.h>
#include <PinChangeInt.h>
#include "config.h"

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1); Serial.flush(); }
  #define DEBUGln(input) {Serial.println(input); delay(1); Serial.flush(); }
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef BLIND
  #define BLINK(delay);
#else
  #define BLINK(delay) {Blink(LED,delay);}
#endif

enum { MEASURE, REPORT, TASK_END };
static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static byte reportCount = REPORT_EVERY;

SPIFlash flash(8, FLASH_JEDEC); //EF40 for 16mbit windbond chip
RFM69 radio;

#ifdef DALLAS_PIN
OneWire oneWire(DALLAS_PIN);
#endif

typedef struct {
  int           vcc;
  int           temp = 85;
  boolean       motionDetected = false;
  boolean       processed = false;
  byte          failedReport = 0;
  byte          locked = 0;
  byte          uptime = 0;
  unsigned int  token = 0;
  int16_t       rssi = 0;
  unsigned short version = VERSION;
} Payload;
Payload lData;

typedef struct {
  unsigned int token = 0;
  byte         cmd = 0;
} ACKPayload;
ACKPayload ackData;

static int smoothedAverage(int prev, int next, bool firstTime = 0)
{
  if (firstTime) return next;
  return (int)((prev + next) / 2);
}

int readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  Sleepy::loseSomeTime(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return (int)result;
}

#ifndef BLIND
void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
#endif

#ifdef DALLAS_PIN
int readDS18B20(OneWire ds)
{
  int HighByte, LowByte, TReading, SignBit, Tc_100;
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  int resultTempFloat;

  Sleepy::loseSomeTime(750);     // allow 750mS for ds18b20 sensor reading to stabilise

  ds.search(addr);
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      return 0.0;                      // CRC is not valid!
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);                   // Start conversion, with parasite power on at the end

  Sleepy::loseSomeTime(1000);     // allow 750mS for ds18b20 sensor reading to stabilise

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);                     // Read Scratchpad

  for ( i = 0; i < 9; i++) {          // we need 9 bytes
    data[i] = ds.read();
  }
  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;        // Test most sig bit
  if (SignBit)                        // Negative
  {
    TReading = (TReading ^ 0xffff) + 1;    // Take 2's comp
  }

  ds.reset_search();
  resultTempFloat =  (int) ((6 * TReading) + TReading / 4);  // Multiply by (100 * 0.0625) or 6.25
  return resultTempFloat;
}
#endif

void process_r_cmd(byte cmd)
{
  DEBUG("Got remote command:"); DEBUGln(cmd);
  switch (cmd)
  {
    case 10:
      {
        uint32_t now = millis();
        while (millis() - now < 1000*5)
        {
          if (radio.receiveDone())
          {
            flash.wakeup();
            CheckForWirelessHEX(radio, flash, true);
            flash.sleep();
          }
        }
      }
      break;
    case 255:
      {
        wdt_enable(WDTO_15MS);
        while(1);
      }
      break;
  }
}

void doReport()
{
  lData.processed = 0;
  lData.token = (unsigned int) random(1, 65536);
  if (radio.sendWithRetry(GATEWAYID, (const void*)(&lData), sizeof(lData), SEND_RETRIES, ACK_TIME))
  {
    ackData = *(ACKPayload*) radio.DATA;
    if (ackData.token && ackData.token == lData.token)
    {
      DEBUGln("Report sent");
      lData.processed = 1;
      lData.failedReport = 0;
      if (ackData.cmd) process_r_cmd(ackData.cmd);
    }
    else DEBUGln("Wrong security token!");
  }
  if (!lData.processed)
  {
    if (lData.failedReport < 255) lData.failedReport++;
    DEBUGln("Failed report");
  }

  lData.rssi = radio.readRSSI();
  radio.sleep();
  BLINK(3);
}

#ifdef MOTIONPIN
void disable_motion()
{
  pinMode(MOTIONPIN, OUTPUT);
  PCintPort::detachInterrupt(MOTIONPIN);
}

void motionIRQ()
{
  if (!lData.motionDetected)
  {
    lData.motionDetected = true;
    disable_motion();
    doReport();
    DEBUGln("Motion detected");
  }
}

void enable_motion()
{
  pinMode(MOTIONPIN, INPUT_PULLUP);
  PCintPort::attachInterrupt(MOTIONPIN, motionIRQ, RISING);
}
#endif

void doMeasure()
{
  DEBUGln("doMeasure()");
  bool firstTime = 0;
  if (lData.uptime == 0)
  {
    firstTime = 1;
  }
  lData.vcc = smoothedAverage(lData.vcc, readVcc(), firstTime);

  lData.uptime++;
  if (lData.uptime == 255) lData.uptime = 50;

  #ifdef DALLAS_PIN
    int temp_ds;

    pinMode(DALLAS_PIN_PWR,OUTPUT);
    digitalWrite(DALLAS_PIN_PWR,HIGH);

    int t = readDS18B20(oneWire);

    digitalWrite(DALLAS_PIN_PWR,LOW);
    pinMode(DALLAS_PIN_PWR, INPUT);
    if (t == 85)
    {
      temp_ds = lData.temp;
    }
    else
    {
      temp_ds = t;
    }
    lData.temp = smoothedAverage(lData.temp, temp_ds, firstTime);
  #endif
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  DEBUGln("Setup...");
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.sleep();

  flash.wakeup();
  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
  {  
    DEBUGln("SPI Flash Init FAIL! (is chip present?)");
  }
  
  flash.sleep();
  
  randomSeed(analogRead(0));
  
  DEBUG("Node id:");DEBUGln(NODEID);
  DEBUG("Sizeof Payload:");DEBUGln(sizeof(lData));
  BLINK(10);
  
  Sleepy::loseSomeTime(50);
  #ifdef MOTIONPIN
    DEBUGln("Motion pin enabled");
    enable_motion();
  #endif
  #ifdef DALLAS_PIN
    DEBUGln("Dallas pin enabled");
  #endif
  
  scheduler.timer(MEASURE, MEASURE_PERIOD); //Run measure scheduler after setup
}

void loop()
{ 
  switch (scheduler.pollWaiting()) {
    
    case MEASURE:
      scheduler.timer(MEASURE, MEASURE_PERIOD);
      #ifdef MOTIONPIN
        if (lData.processed && lData.motionDetected)
        {
          DEBUGln("Cleaning motion sensor");
          enable_motion();
          lData.motionDetected = false;
        }
      #endif
      doMeasure();
      if (++reportCount >= REPORT_EVERY) {
        reportCount = 0;
        scheduler.timer(REPORT, 0);
      }
      break;
      
    case REPORT:
      doReport();
      break;
  }
}
