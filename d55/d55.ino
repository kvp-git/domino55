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
#define LED_NONE   (-1)

// ---- TURNOUTS ----

struct TurnoutDescription
{
  int leds[2]; // straight,diverging
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
  int leds[3]; // red,green,white
  Driver driver;
};

struct SignalState
{
  int state;
  int image[2]; // default,blinking
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

#define SHOW_DARK         (0x00)
#define SHOW_RED          (0x04)
#define SHOW_GREEN        (0x01)
#define SHOW_YELLOW       (0x02)
#define SHOW_GREENYELLOW  (0x09)
#define SHOW_YELLOWYELLOW (0x0A)
#define SHOW_CALLING      (0x14)
#define SHOW_TESTALL      (0x1F)

#define TURNOUTS_NUM      (3)
#define SIGNALS_NUM       (6)
#define ROUTELOCKS_NUM    (3)

Turnout turnouts[TURNOUTS_NUM] =
{
  // turnout 1
  {
    {{41-LED_FROM, 39-LED_FROM},{DRIVER_TURNOUT,0x02,0x00}},{}
  },
  // turnout 2
  {
    {{36-LED_FROM, 34-LED_FROM},{DRIVER_TURNOUT,0x07,0x00}},{}
  },
  // turnout 3
  {
    {{37-LED_FROM, 35-LED_FROM},{DRIVER_TURNOUT,0x07,0x02}},{}
  },
};

Signal signals[SIGNALS_NUM] =
{
  {{{45-LED_FROM,47-LED_FROM,49-LED_FROM},{DRIVER_SIGNAL4,0x01,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
  {{{42-LED_FROM,44-LED_FROM,46-LED_FROM},{DRIVER_SIGNAL3,0x03,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
  {{{51-LED_FROM,38-LED_FROM,40-LED_FROM},{DRIVER_SIGNAL4,0x04,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
  {{{25-LED_FROM,50-LED_FROM,48-LED_FROM},{DRIVER_SIGNAL4,0x05,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
  {{{31-LED_FROM,29-LED_FROM,27-LED_FROM},{DRIVER_SIGNAL3,0x06,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
  {{{28-LED_FROM,26-LED_FROM,24-LED_FROM},{DRIVER_SIGNAL4,0x08,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
};

RouteLock routeLocks[ROUTELOCKS_NUM] =
{
  {{43-LED_FROM,LED_NONE   },{ROUTESTATE_IDLE,-1}},
  {{32-LED_FROM,30-LED_FROM},{ROUTESTATE_IDLE,-1}},
  {{33-LED_FROM,LED_NONE   },{ROUTESTATE_IDLE,-1}},
};

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

void sendCommand(uint8_t addr, uint8_t data, bool debug)
{
  uint8_t msg[3];
  msg[0] = (addr & 127) | 128;
  msg[1] = data & 127;
  msg[2] = (-(msg[0] + msg[1])) & 127;
  Serial1.write(msg, sizeof(msg));
  if (debug)
  {
    Serial.print("(");
    Serial.print(addr);
    Serial.print(",0x");
    Serial.print((data >> 4) & 15, HEX);
    Serial.print(data & 15, HEX);
    Serial.print(") ");
  }
}

void sendCommands(bool debug)
{
  for (int t = 1; t < 9; t++)
    sendCommand(t, dataOut[t], debug);
  if (debug)
    Serial.println("");
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
  if ((turnoutNum < 0) || (turnoutNum >= TURNOUTS_NUM))
    return;
  turnouts[turnoutNum].state.state = TURNOUTSTATE_MOVING_TURNOUT;
  turnouts[turnoutNum].state.counter = TURNOUT_MOVE_TIME;
  turnouts[turnoutNum].state.route = route;
  turnouts[turnoutNum].state.routeSelectionNum = ROUTE_NONE;
}

void turnoutStop(int turnoutNum)
{
  if ((turnoutNum < 0) || (turnoutNum >= TURNOUTS_NUM))
    return;
  turnouts[turnoutNum].state.state = TURNOUTSTATE_IDLE;
  turnouts[turnoutNum].state.counter = 0;
}

void signalSet(int signalNum, int state)
{
  if ((signalNum < 0) || (signalNum >= SIGNALS_NUM))
    return;
  signals[signalNum].state.state = state;
}

bool routeLock(int routeNum, int routeSelectionNum)
{
  if ((routeNum < 0) || (routeNum >= ROUTELOCKS_NUM))
    return false;
  if (routeLocks[routeNum].state.state != ROUTESTATE_IDLE)
    return false;
  routeLocks[routeNum].state.state = ROUTESTATE_ACTIVE;
  routeLocks[routeNum].state.routeSelectionNum = routeSelectionNum;
  return true;
}

bool routeClear(int routeNum)
{
  if ((routeNum < 0) || (routeNum >= ROUTELOCKS_NUM))
    return false;
  if (routeLocks[routeNum].state.state != ROUTESTATE_ACTIVE)
    return false;
  routeLocks[routeNum].state.state = ROUTESTATE_IDLE;
  routeLocks[routeNum].state.routeSelectionNum = ROUTE_NONE;
  return true;
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
  for (int t = 0; t < SIGNALS_NUM; t++)
  {
    int address = signals[t].desc.driver.address;
    int offset = signals[t].desc.driver.offset;
    switch (signals[t].state.state)
    {
      case SIGNAL_STOP:
        leds[signals[t].desc.leds[0]] = LEDS_ON;
        leds[signals[t].desc.leds[1]] = LEDS_OFF;
        leds[signals[t].desc.leds[2]] = LEDS_OFF;
        break;
      case SIGNAL_GO:
        leds[signals[t].desc.leds[0]] = LEDS_OFF;
        leds[signals[t].desc.leds[1]] = LEDS_ON;
        leds[signals[t].desc.leds[2]] = LEDS_OFF;
        break;
      case SIGNAL_CALL:
        leds[signals[t].desc.leds[0]] = LEDS_ON;
        leds[signals[t].desc.leds[1]] = LEDS_OFF;
        leds[signals[t].desc.leds[2]] = LEDS_ON;
        break;
    }
    dataOut[address] = signals[t].state.image[blinkStep];
  }
}

void updateRoutes()
{
  for (int t = 0; t < ROUTELOCKS_NUM; t++)
  {
    leds[routeLocks[t].desc.led] = ((routeLocks[t].state.state == ROUTESTATE_ACTIVE) ? LEDS_ON : LEDS_OFF);
    if (routeLocks[t].desc.crossingLed >= 0)
      leds[routeLocks[t].desc.crossingLed] = ((routeLocks[t].state.state == ROUTESTATE_ACTIVE) ? LEDS_BLINK : LEDS_OFF);
  }
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
  unsigned long dataOutT1 = millis() / 100; // 10 msg/sec
  if (dataOutT1 != dataOutT0)
  {
    dataOutT0 = dataOutT1;
    if (initStep > 1)
    {
      updateTurnouts();
      updateSignals();
      updateRoutes();
      if (modeFlag == MODE_RUN)
      {
        dataOut[2] = (dataOut[2] & 0x0f) | (keys[0] ? 0x10 : 0x00); // station lights on switch 0
        dataOut[7] = (dataOut[7] & 0x0f) | ((routeLocks[2].state.state == ROUTESTATE_ACTIVE) ? 0x10 : 0x00); // crossing light signal
      }
    }
    sendCommands((dataOutT1 % 10) == 0);
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
    for (int t = 0; t < SIGNALS_NUM; t++)
      signalSet(t, SIGNAL_CALL);
    for (int t = 0; t < ROUTELOCKS_NUM; t++)
      routeLock(t, -1);
  }
  if ((initTime >= 7000) && (initStep == 2))
  {
    initStep = 3;
    Serial.println("turnout reset step 2...");
    for (int t = 0; t < TURNOUTS_NUM; t++)
      turnoutSet(t, TURNOUT_STRAIGHT);
    for (int t = 0; t < SIGNALS_NUM; t++)
      signalSet(t, SIGNAL_GO);
  }
  if ((initTime >= 12000) && (initStep == 3))
  {
    for (int t = 0; t < SIGNALS_NUM; t++)
      signalSet(t, SIGNAL_STOP);
    for (int t = 0; t < ROUTELOCKS_NUM; t++)
      routeClear(t);
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

