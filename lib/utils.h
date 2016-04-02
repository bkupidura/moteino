#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1); Serial.flush(); }
  #define DEBUGln(input) {Serial.println(input); delay(1); Serial.flush(); }
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

enum { MSG_ERROR=0, MSG_NULL=1, MSG_MEASURE=2, MSG_COMMAND=3};
enum { CMD_MEASURE=1, CMD_UPDATE=10, CMD_BLINK=11, CMD_REBOOT=255};

#define PAYLOADL3_LEN 6 //size of PayloadL3 in bytes without data[] rounded up; (16b+16b+8b+8b)/8
typedef struct {
  unsigned int  token = 0; //Used by sender to verify ack
  unsigned int  id = 0; //Used by recipient to verify packet
  byte          type = MSG_NULL; //Type of packet
  byte          size; //size of payload
  byte          data[RF69_MAX_DATA_LEN-PAYLOADL3_LEN]; //real payload
} PayloadL3;

typedef struct {
  PayloadL3 payload;
  uint8_t   senderid;
} PayloadL3ws;

typedef struct {
  int           vcc;
  int           temp = 85; //DS18B20 returns 85 in case of error
  boolean       motionDetected = false;
  byte          failedReport = 0;
  byte          uptime = 0;
  int16_t       rssi = 0;
} PayloadL4measure;

typedef struct {
  byte cmd;
} PayloadL4cmd;

typedef struct {
  byte nodeid;
  byte cmd; 
} remote_cmd;

PayloadL3 sendWithRetry(RFM69 radio, uint8_t toAddress, PayloadL3 lDataL3, uint8_t retries, uint8_t retryWaitTime);
bool check_id(unsigned int remote_id, unsigned int *local_id);
void Blink(byte PIN, int DELAY_MS);

PayloadL3 sendWithRetry(RFM69 radio, uint8_t toAddress, PayloadL3 lDataL3, uint8_t retries, uint8_t retryWaitTime){
  PayloadL3 ackDataL3;
  for (uint8_t i = 0; i <= retries; i++){
    if (radio.sendWithRetry(toAddress, (const void*)(&lDataL3), sizeof(lDataL3), 1, retryWaitTime)){
      ackDataL3 = *(PayloadL3*) radio.DATA;
      if (ackDataL3.token && ackDataL3.token == lDataL3.token){
        if (ackDataL3.type != MSG_ERROR){
          return ackDataL3;
        } else {
          lDataL3.id = ackDataL3.id;
        }
      }
    }
  }
  ackDataL3.type = MSG_ERROR;
  return ackDataL3;
}
bool check_id(unsigned int remote_id, unsigned int *local_id){
  if (remote_id != *local_id)
    return false;
  else
    *local_id = (unsigned int) random(0, 65536);
  return true;
}
void Blink(byte PIN, int DELAY_MS){
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
