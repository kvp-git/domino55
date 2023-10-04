// Integra Domino 55 emulator for a serial bus based control by KVP in 2023

#define LED_FROM (24)
#define LED_TO   (51)
#define LED_NUM  (28)

#define KEY_FROM (A0)
#define KEY_TO   (A15)
#define KEY_NUM  (16) // only using the 1st 15

#define BOARD_LED (13)

#define TURNOUT_MOVE_TIME (40)

enum MODE_FLAGS
{
  MODE_INIT = 0,
  MODE_RUN = 1,
  MODE_TEST = 2,
};

int modeFlag = MODE_INIT;
unsigned long initT0 = 0;
int initStep = 0;

bool keys[KEY_NUM] = {};
byte leds[LED_NUM] = {};
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

// ---- ROUTELOCKS ----

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

struct RouteLock
{
  RouteLockDescription desc;
  RouteLockState state;  
};

#define ROUTE_NONE (-1)

// ---- TURNOUTS ----

struct TurnoutDescription
{
  int leds[2];
  Driver driver;
};

struct TurnoutState
{
  int state;
  int counter;
  int route;
  int routeSelectionNum;
};

struct Turnout
{
  TurnoutDescription desc;
  TurnoutState state;
};

enum TURNOUTSTATE
{
  TURNOUTSTATE_IDLE = 0,
  TURNOUTSTATE_MOVING_TURNOUT,
  TURNOUTSTATE_MOVING_ROUTE,
  TURNOUTSTATE_LOCKED,
};

enum TURNOUTROUTE
{
  TURNOUT_STRAIGHT,
  TURNOUT_DIVERGING,
};

// ---- SIGNALS ----

struct SignalDescription
{
  int leds[4];
  Driver driver;
};

struct SignalState
{
  int state;
};

struct Signal
{
  SignalDescription desc;
  SignalState state;
};

enum DRIVERTYPE
{
  DRIVER_TURNOUT, // 2 outputs, time pulsed
  DRIVER_SIGNAL2, // 2 outputs, on/off/blink
  DRIVER_SIGNAL3, // 3 outputs, on/off/blink
  DRIVER_SIGNAL4, // 4 outputs, on/off/blink
  DRIVER_SIGNAL5, // 5 outputs, on/off/blink
  DRIVER_CROSSING, // 1 output, on/off
  DRIVER_LIGHTS, // 1 output, on/off
};

enum ROUTESTATE
{
  ROUTESTATE_IDLE = 0,
  ROUTESTATE_ACTIVE,
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
#define SHOW_CALLING      (0x14)

#define TURNOUTS_NUM      (3)
#define SIGNALS_NUM       (6)
#define ROUTELOCKS_NUM    (2)

Turnout turnouts[TURNOUTS_NUM] =
{
  // turnout 1
  {
    {{41-LED_FROM, 39-LED_FROM},{DRIVER_TURNOUT,0x02,0x00}},
    {}
  },
  // turnout 2
  {
    {{36-LED_FROM, 34-LED_FROM},{DRIVER_TURNOUT,0x07,0x00}},
    {}
  },
  // turnout 3
  {
    {{37-LED_FROM, 35-LED_FROM},{DRIVER_TURNOUT,0x07,0x02}},
    {}
  },
};

Signal signals[SIGNALS_NUM];
RouteLock routeLocks[ROUTELOCKS_NUM];

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
  Serial1.write(msg, sizeof(msg));
  /*Serial.print("(");
  Serial.print(addr);
  Serial.print(",0x");
  Serial.print(data, HEX);
  Serial.print(") ");*/
}

void sendCommands()
{
  for (int t = 1; t < 9; t++)
    sendCommand(t, dataOut[t]);
  //Serial.println("");
}

void readKeys()
{
  for (int t = 0; t < KEY_NUM; t++)
    keys[t] = (digitalRead(KEY_FROM + t) == LOW) ? true : false;
}

void writeLeds()
{
  blinkStep = ((millis() / 500) & 1);
  int ledsMask = ((blinkStep == 0) ? 1 : 2);
  for (int t = 0; t < LED_NUM; t++)
    digitalWrite(LED_FROM + t, (((leds[t] & ledsMask) != 0) ? HIGH : LOW));
}

void turnoutSet(int turnoutNum, int route)
{
  if ((turnoutNum < 0) || (turnoutNum > TURNOUTS_NUM))
    return;
  turnouts[turnoutNum].state.state = TURNOUTSTATE_MOVING_TURNOUT;
  turnouts[turnoutNum].state.counter = TURNOUT_MOVE_TIME;
  turnouts[turnoutNum].state.route = route;
  turnouts[turnoutNum].state.routeSelectionNum = ROUTE_NONE;
}

void turnoutStop(int turnoutNum)
{
  if ((turnoutNum < 0) || (turnoutNum > TURNOUTS_NUM))
    return;
  turnouts[turnoutNum].state.state = TURNOUTSTATE_IDLE;
  turnouts[turnoutNum].state.counter = 0;
}

void updateTurnouts()
{
  //Serial.println("updateTurnouts");
  for (int t = 0; t < TURNOUTS_NUM; t++)
  {
    int address = turnouts[t].desc.driver.address;
    int offset = turnouts[t].desc.driver.offset;
    if (turnouts[t].state.counter > 0)
    {
      turnouts[t].state.counter--;
      if (turnouts[t].state.counter == 0)
      {
        turnoutStop(t);
        switch(turnouts[t].state.state)
        {
          case TURNOUTSTATE_IDLE:
          case TURNOUTSTATE_LOCKED:
            break;
          case TURNOUTSTATE_MOVING_TURNOUT:
            turnouts[t].state.state = TURNOUTSTATE_IDLE;
            break;
          case TURNOUTSTATE_MOVING_ROUTE:
            turnouts[t].state.state = TURNOUTSTATE_LOCKED;
            break;
        }
      }
    }
    switch (turnouts[t].state.state)
    {
      case TURNOUTSTATE_IDLE:
      case TURNOUTSTATE_LOCKED:
        dataOut[address] &= (~(3 << offset));
        switch (turnouts[t].state.route)
        {
          case TURNOUT_STRAIGHT:
            leds[turnouts[t].desc.leds[0]] = LEDS_ON;
            leds[turnouts[t].desc.leds[1]] = LEDS_OFF;
            break;
          case TURNOUT_DIVERGING:
            leds[turnouts[t].desc.leds[0]] = LEDS_OFF;
            leds[turnouts[t].desc.leds[1]] = LEDS_ON;
            break;
        }
        break;
      case TURNOUTSTATE_MOVING_TURNOUT:
      case TURNOUTSTATE_MOVING_ROUTE:
        dataOut[address] &= ~(3 << offset);
        dataOut[address] |= (((turnouts[t].state.route == TURNOUT_STRAIGHT) ? 0x01 : 0x02) << offset);
        leds[turnouts[t].desc.leds[0]] = LEDS_BLINK;
        leds[turnouts[t].desc.leds[1]] = LEDS_BLINK;
        break;
    }
  }
}

void updateSignals()
{
  // TODO!!!
}

void initLogic();
void testInit();
void testLogic();
void runningInit();
void runningLogic();

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
  initT0 = millis();
}

void loop()
{
  readKeys();
  switch (modeFlag)
  {
    case MODE_INIT:
      initLogic();
      break;
    case MODE_TEST:
      testLogic();
      break;
    case MODE_RUN:
      runningLogic();
      break;
  }
  writeLeds();
  updateSignals();
  unsigned long dataOutT1 = millis() / 100; // 10 msg/sec
  if (dataOutT1 != dataOutT0)
  {
    dataOutT0 = dataOutT1;
    updateTurnouts();
    sendCommands();
  }
}

void initLogic()
{
  int initTime = millis() - initT0;
  if (initStep == 0)
  {
    initStep = 1;
    Serial.println("led tests...");
    for (int t = 0; t < LED_NUM; t++)
      leds[t] = LEDS_ON;    
  }
  if ((initTime >= 2000) && (initStep == 1))
  {
    initStep = 2;
    for (int t = 0; t < LED_NUM; t++)
      leds[t] = LEDS_OFF;
    Serial.println("turnout reset step 1...");
    for (int t = 0; t < TURNOUTS_NUM; t++)
      turnoutSet(t, TURNOUT_DIVERGING);  
  }
  if ((initTime >= 6000) && (initStep == 2))
  {
    initStep = 3;
    Serial.println("turnout reset step 2...");
    for (int t = 0; t < TURNOUTS_NUM; t++)
      turnoutSet(t, TURNOUT_STRAIGHT);
  }
  if ((initTime >= 10000) && (initStep == 3))
  {
    Serial.println("mode selection");
    if (keys[0])
    {
      modeFlag = MODE_TEST;
      testInit();
    }else
    {
      modeFlag = MODE_RUN;
      runningInit();
    }
  }
}

// ---- RUNNING ----

void runningInit()
{
  for (int t = 0; t < LED_NUM; t++)
    leds[t] = LEDS_OFF;
  for (int t = 0; t < TURNOUTS_NUM; t++)
  {
    turnouts[t].state.state = TURNOUTSTATE_IDLE;
    turnouts[t].state.counter = 0;
    turnouts[t].state.routeSelectionNum = ROUTE_NONE;
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

void testInit()
{
  for (int t = 0; t < LED_NUM; t++)
    leds[t] = LEDS_BLINK;
}

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

