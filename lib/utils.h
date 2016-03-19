enum { MSG_ERROR=0, MSG_NULL=1, MSG_MEASURE=2, MSG_COMMAND=3};
enum { CMD_MEASURE=1, CMD_UPDATE=10, CMD_BLINK=11, CMD_REBOOT=255};

#define PAYLOADL3_LEN 11 //size of PayloadL3 in bytes rounded up; (1b+8b+32b+32b)/8
#define ACKPAYLOADL3_LEN 10 //size of ACKPayloadL3 in bytes rounded up; (32b+32b+8b)/8
typedef struct {
  bool          processed = false;
  unsigned int  token = 0;
  uint32_t      time = 0;
  byte          type = MSG_NULL;
  byte          size;
  byte          data[RF69_MAX_DATA_LEN-PAYLOADL3_LEN];
} PayloadL3;

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
