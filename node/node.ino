#include <RFM69.h> //get it here: http://github.com/lowpowerlab/rfm69
#include <SPI.h> //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: http://github.com/lowpowerlab/spiflash
#include <Ports.h> //get it here: https://github.com/jcw/jeelib
#include <OneWire.h>       //get it here: https://github.com/pbrook/arduino-onewire
#include <WirelessHEX69.h>
#include <avr/wdt.h>
#include <PinChangeInt.h>
#include "../lib/config.h"
#include "../lib/utils.h"

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

PayloadL3 lDataL3;
PayloadL4measure lDataL4measure;

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

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

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
    case CMD_UPDATE:
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
    case CMD_BLINK:
      {
        Blink(LED, 10000);
      }
      break;
    case CMD_REBOOT:
      {
        wdt_enable(WDTO_15MS);
        while(1);
      }
      break;
  }
}

void build_data()
{
  lDataL3.processed = false;
  lDataL3.token = (unsigned int) random(1, 65536);
  lDataL3.type = MSG_MEASURE;
  lDataL3.size = sizeof(lDataL4measure);
  memcpy(&lDataL3.data, &lDataL4measure, sizeof(lDataL4measure));
}

void doReport()
{
  PayloadL3 ackDataL3;
  PayloadL4cmd ackDataL4cmd;
  build_data();

  if (radio.sendWithRetry(GATEWAYID, (const void*)(&lDataL3), sizeof(lDataL3), SEND_RETRIES, ACK_TIME))
  {
    ackDataL3 = *(PayloadL3*) radio.DATA;
    if (ackDataL3.token && ackDataL3.token == lDataL3.token)
    {
      if (ackDataL3.type != MSG_ERROR)
      {
        DEBUGln("Report sent");
        lDataL3.processed = true;
        lDataL4measure.failedReport = 0;
      }
      if (ackDataL3.type == MSG_COMMAND)
      {
        memcpy(&ackDataL4cmd, &ackDataL3.data, ackDataL3.size);
        process_r_cmd(ackDataL4cmd.cmd);
      }
      if (ackDataL3.time) lDataL3.time = ackDataL3.time;
    }
  }
  if (!lDataL3.processed)
  {
    if (lDataL4measure.failedReport < 255) lDataL4measure.failedReport++;
    DEBUGln("Failed report");
  }

  lDataL4measure.rssi = radio.readRSSI();
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
  if (!lDataL4measure.motionDetected)
  {
    lDataL4measure.motionDetected = true;
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
  if (lDataL4measure.uptime == 0)
  {
    firstTime = 1;
  }
  lDataL4measure.vcc = smoothedAverage(lDataL4measure.vcc, readVcc(), firstTime);

  lDataL4measure.uptime++;
  if (lDataL4measure.uptime == 255) lDataL4measure.uptime = 50;

  #ifdef DALLAS_PIN
    int temp_ds;

    pinMode(DALLAS_PIN_PWR,OUTPUT);
    digitalWrite(DALLAS_PIN_PWR,HIGH);

    int t = readDS18B20(oneWire);

    digitalWrite(DALLAS_PIN_PWR,LOW);
    pinMode(DALLAS_PIN_PWR, INPUT);
    if (t == 85)
    {
      temp_ds = lDataL4measure.temp;
    }
    else
    {
      temp_ds = t;
    }
    lDataL4measure.temp = smoothedAverage(lDataL4measure.temp, temp_ds, firstTime);
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
  Blink(LED, 10);
  
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
        if (lDataL3.processed && lDataL4measure.motionDetected)
        {
          DEBUGln("Cleaning motion sensor");
          enable_motion();
          lDataL4measure.motionDetected = false;
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
