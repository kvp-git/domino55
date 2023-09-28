// Integra Domino 55 emulator for a serial bus based control by KVP in 2023

#define LED_FROM (24)
#define LED_TO   (51)
#define LED_NUM  (28)

#define KEY_FROM (A0)
#define KEY_TO   (A15)
#define KEY_NUM  (16) // only using the 1st 15

#define BOARD_LED (13)

enum MODE_FLAGS
{
  MODE_INIT = -1,
  MODE_RUN = 0,
  MODE_TEST = 1,
};

int modeFlag = MODE_INIT;

bool keys[KEY_NUM] = {};
byte leds[LED_NUM] = {1,1,1,2,2,2,3,3,3};
bool blinkStep = 0;

#define LEDS_OFF   (0)
#define LEDS_BLINK (1)
#define LEDS_ON    (3)

uint8_t dataOut[9] = {0,0,0,0,0,0,0,0,0};
unsigned long dataOutT0 = 0;

struct Driver
{
  int driverType;
  int address;
  int offset;
  int count;
};

struct RouteSelection
{
  int lockKeys[2];
  int unlockKeys[2];
  int turnoutNums[2];
  int turnoutDirs[2];
  int routeLockNum;
  int signalNum;
  int ledNum[2];
};

struct TurnoutSelection
{
  int changeKeys[2];
  int turnoutNum;
};

struct SignalSelection
{
  int callKeys[2];
  int signalNum;
};

struct RouteLockDescription
{
  int led;
  int crossingLed;
};

struct RouteLockState
{
  int state;
  int routeSelectionNum;
};

struct TurnoutDescription
{
  int leds[2];
  Driver driver;
};

struct TurnoutState
{
  int state;
  int counter;
  int routeSelectionNum;
};

struct SignalDescription
{
  int leds[4];
  Driver driver;
};

struct SignalState
{
  int state;
};

struct Turnout
{
  TurnoutDescription desc;
  TurnoutState state;
};

struct Signal
{
  SignalDescription desc;
  SignalState state;
};

enum DRIVERTYPE
{
  DRIVER_TURNOUT,
  DRIVER_SIGNAL2,
  DRIVER_SIGNAL3,
  DRIVER_SIGNAL4,
  DRIVER_SIGNAL5,
  DRIVER_CROSSING,
  DRIVER_LIGHTS,
};

enum ROUTESTATE
{
  ROUTESTATE_IDLE = 0,
  ROUTESTATE_ACTIVE,
};

enum TURNOUTSTATE
{
  TURNOUTSTATE_IDLE = 0,
  TURNOUTSTATE_MOVING_TURNOUT,
  TURNOUTSTATE_MOVING_ROUTE,
  TURNOUTSTATE_LOCKED,
};

enum SIGNALSTATE
{
  SIGNAL_STOP = 0,
  SIGNAL_GO,
  SIGNAL_CALL,
};

#define SHOW_RED          (0x04)
#define SHOW_GREEN        (0x01)
#define SHOW_YELLOW       (0x02)
#define SHOW_GREENYELLOW  (0x09)
#define SHOW_YELLOWYELLOW (0x0A)
#define SHOW_CALLING      (0x11)
/*
  send_cmd(0x02, (stTo1 ? 0x02 : 0x01) | (stLights ? 0x10 : 0x00));  
  send_cmd(0x07, (stTo2 ? (0x01 | 0x08) : (0x02 | 0x04)) | ((!stIn2 && !stOut2) ? 0x10 : 0x00));
  send_cmd(0x01, headIn1);
  send_cmd(0x03, headOut1a);
  send_cmd(0x04, headOut1b);
  send_cmd(0x06, headOut2a);
  send_cmd(0x05, headOut2b);
  send_cmd(0x08, headIn2);
*/

void sendCommand(uint8_t addr, uint8_t data)
{
  uint8_t msg[3];
  msg[0] = (addr & 127) | 128;
  msg[1] = data & 127;
  msg[2] = (-(msg[0] + msg[1])) & 127;
  Serial.print("(0x");
  Serial.print(addr, HEX);
  Serial.print(" 0x");
  Serial.print(data, HEX);
  Serial.print(") ");
  Serial1.write(msg, sizeof(msg)); 
}

void setup()
{
  Serial.begin(115200); // usb console
  Serial1.begin(9600);  // command bus

  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LOW);

  for (int t = LED_FROM; t <= LED_TO; t++)
  {
    pinMode(t, OUTPUT);
    digitalWrite(t, LOW);
  }
  for (int t = KEY_FROM; t <= KEY_TO; t++)
    pinMode(t, INPUT_PULLUP);

  Serial.println();
  Serial.println();
  Serial.println("Domino 55 v0.1 by KVP in 2023");
  Serial.println();
}

void testLogic();
void runningLogic();

void loop()
{
  for (int t = 0; t < KEY_NUM; t++)
    keys[t] = (digitalRead(KEY_FROM + t) == LOW) ? true : false;

  if (modeFlag == MODE_INIT)
  {
    if (keys[0])
       modeFlag = MODE_TEST;
    else
       modeFlag = MODE_RUN;
  }

  if (modeFlag == MODE_TEST) // test mode
  {
    testLogic();
  } else // running mode
  {
    runningLogic();
  }

  blinkStep = ((millis() / 500) & 1);
  int ledsMask = ((blinkStep == 0) ? 1 : 2);
  for (int t = 0; t < LED_NUM; t++)
    digitalWrite(LED_FROM + t, (((leds[t] & ledsMask) != 0) ? HIGH : LOW));

  unsigned long dataOutT1 = millis() / 100; // 10 msg/sec
  if (dataOutT1 != dataOutT0)
  {
    dataOutT0 = dataOutT1;
    for (int t = 1; t < 9; t++)
      sendCommand(t, dataOut[t]);
    Serial.println("");
  }
}

void runningLogic()
{
  
}

// ---------- TEST ----------

unsigned long testT0 = 0;
bool testV1 = false;
bool testV2 = false;
bool testV3 = false;
bool testCR = false;

void testLogic()
{
  unsigned long testT1 = millis() / 1000;
  if (testT1 == testT0)
    return;
  testT0 = testT1;
  Serial.print("TEST: ");
  for (int t = 0; t < KEY_NUM; t++)
    Serial.print((keys[t] ? 'x' : '_'));
  Serial.println("");
  if (keys[10])
  {
    dataOut[1] = (dataOut[1] + 1) & 31;
    dataOut[3] = (dataOut[3] + 1) & 31;
    dataOut[4] = (dataOut[4] + 1) & 31;
    dataOut[5] = (dataOut[5] + 1) & 31;
    dataOut[6] = (dataOut[6] + 1) & 31;
    dataOut[8] = (dataOut[8] + 1) & 31;
  }
  if (keys[12])
    testV2 = (testV2 ? false : true);
  if (keys[11])
    testV3 = (testV3 ? false : true);
  if (keys[3])
    testV1 = (testV1 ? false : true);
  if (keys[10])
    testCR = (testCR ? false : true);
  
  if (testV1)
    dataOut[2] = 0x02 | 0x10;
  else
    dataOut[2] = 0x01;
  uint8_t v = 0;
  if (testV2)
    v |= 0x02;
  else
    v |= 0x01;
  if (testV3)
    v |= 0x08;
  else
    v |= 0x04;
  if (testCR)
    v |= 0x10;
  dataOut[7] = v;
}

