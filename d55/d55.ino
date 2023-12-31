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
  int turnoutNums[2];
  int turnoutDirs[2];
  int routeLockNums[2];
  int signalNum;
  int blockLock;
  int blockTurnout;
  int blockTurnoutDir;
};

struct TurnoutSelection
{
  int changeKeys[2];
  int turnoutNum;
  int routeLockNum;
};

struct SignalSelection
{
  int callKeys[2];
  int clearKeys[2];
  int signalNum;
  int routeLockNums[2];
};

// ---- ROUTELOCKS ----

struct RouteLockDescription
{
  int unlockKeys[2];
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

#define ROUTE_NONE   (-1)
#define UNUSED       (-1)

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
  TURNOUT_STRAIGHT = 0,
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
  ROUTESTATE_LOCKED,
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
#define SHOW_BLANKYELLOW  (0x08)
#define SHOW_YELLOWYELLOW (0x0A)
#define SHOW_CALLING      (0x14)
#define SHOW_TESTALL      (0x1F)

#define TURNOUTS_NUM      (3)
#define SIGNALS_NUM       (6)
#define ROUTELOCKS_NUM    (3)
#define ROUTES_NUM        (8)

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

RouteLock routeLocks[ROUTELOCKS_NUM] = // unlockKeys[2],led,crossingLed
{
  {{{ 2,7},43-LED_FROM,UNUSED     },{ROUTESTATE_IDLE,ROUTE_NONE}},
  {{{13,7},32-LED_FROM,30-LED_FROM},{ROUTESTATE_IDLE,ROUTE_NONE}},
  {{{13,7},33-LED_FROM,UNUSED     },{ROUTESTATE_IDLE,ROUTE_NONE}},
};

RouteSelection routeSelections[ROUTES_NUM] = // lockKeys[2],turnoutNums[2],turnoutDirs[2],routeLockNums[2],signalNum
{
  {{1,8},{0,UNUSED},{TURNOUT_STRAIGHT, UNUSED},{0,UNUSED},0,1,1,TURNOUT_STRAIGHT},
  {{1,9},{0,UNUSED},{TURNOUT_DIVERGING,UNUSED},{0,UNUSED},0,1,1,TURNOUT_DIVERGING},
  {{5,2},{0,UNUSED},{TURNOUT_STRAIGHT, UNUSED},{0,UNUSED},1,UNUSED,UNUSED,UNUSED},
  {{6,2},{0,UNUSED},{TURNOUT_DIVERGING,UNUSED},{0,UNUSED},2,UNUSED,UNUSED,UNUSED},

  {{14,5},{1,2     },{TURNOUT_STRAIGHT, TURNOUT_STRAIGHT}, {1,2},5,0,0,TURNOUT_STRAIGHT},
  {{14,6},{1,2     },{TURNOUT_DIVERGING,TURNOUT_DIVERGING},{1,2},5,0,0,TURNOUT_DIVERGING},
  {{8,13},{1,2     },{TURNOUT_STRAIGHT, TURNOUT_STRAIGHT}, {1,2},4,UNUSED,UNUSED,UNUSED},
  {{9,13},{1,2     },{TURNOUT_DIVERGING,TURNOUT_DIVERGING},{1,2},3,UNUSED,UNUSED,UNUSED},
};

TurnoutSelection turnoutSelections[TURNOUTS_NUM] = // changeKeys[2],turnoutNum,routeLockNum
{
  {{3,4},0,0},
  {{12,4},1,1},
  {{11,4},2,2},
};

SignalSelection signalSelections[SIGNALS_NUM] = // callKeys[2],clearKeys[2],signalNum,routeLockNums[2]
{
  {{ 1,10},{ 1,7},0,{0,UNUSED}},
  {{ 5,10},{ 5,7},1,{0,UNUSED}},
  {{ 6,10},{ 6,7},2,{0,UNUSED}},
  {{ 8,10},{ 8,7},4,{1,2}},
  {{ 9,10},{ 9,7},3,{1,2}},
  {{14,10},{14,7},5,{1,2}},
};

/*  --Trs--s-\
  s-rT--s--s--Tr-s

s:signal (R,G,W)
r:route (-,a,L)
c:crossing ( ,*)
T:turnout (-,/,\)
*/

char debugSignalToChar(int signalNum)
{
  if ((signalNum < 0) || (signalNum >= SIGNALS_NUM))
    return;
  switch (signals[signalNum].state.state)
  {
    case SIGNAL_STOP:
      return 'R';
    case SIGNAL_GO:
      return 'G';
    case SIGNAL_CALL:
      return 'W';
    default:
      return 'x';
  }
}

char debugRouteToChar(int routeNum)
{
  if ((routeNum < 0) || (routeNum >= ROUTELOCKS_NUM))
    return false;
  switch (routeLocks[routeNum].state.state)
  {
    case ROUTESTATE_IDLE:
      return '-';
    case ROUTESTATE_ACTIVE:
      return 'a';
    case ROUTESTATE_LOCKED:
      return 'L';
    default:
      return 'x';
  }
}

char debugTurnoutToChar(int turnoutNum, char diverging)
{
  switch (turnouts[turnoutNum].state.state)
  {
    case TURNOUTSTATE_IDLE:
    case TURNOUTSTATE_LOCKED:
      if (turnouts[turnoutNum].state.route == 0)
        return '-';
      else
        return diverging;
    case TURNOUTSTATE_MOVING_TURNOUT:
    case TURNOUTSTATE_MOVING_ROUTE:
      return ':';
    default:
      return '!';
  }
}

const char hexTbl[] = "0123456789ABCDEF";

char num2hex(int v)
{
  return hexTbl[v & 15];
}

void signalToText(int signalNum, char* buf)
{
  switch (signals[signalNum].state.image[0])
  {
    case SHOW_DARK:
      buf[0] = 'x';
      buf[1] = 'x';
      break;
    case SHOW_RED:
      buf[0] = 'R';
      buf[1] = ' ';
      break;
    case SHOW_GREEN:
      buf[0] = 'G';
      buf[1] = ' ';
      break;
    case SHOW_YELLOW:
      buf[0] = 'Y';
      buf[1] = ' ';
      break;
    case SHOW_GREENYELLOW:
      buf[0] = 'G';
      buf[1] = 'Y';
      break;
    case SHOW_YELLOWYELLOW:
      buf[0] = 'Y';
      if (signals[signalNum].state.image[1] == SHOW_BLANKYELLOW)
        buf[1] = 'y';
      else
        buf[1] = 'Y';
      break;
    case SHOW_CALLING:
      buf[0] = 'R';
      buf[1] = 'W';
      break;
    case SHOW_TESTALL:
      buf[0] = 'T';
      buf[1] = 'T';
      break;
  }
}

void debugImage()
{
  char text[4][24];
  strcpy(text[0], " c|-Trs--s-\\");
  strcpy(text[1], "s-rT--s--s--Tr-s");
  strcpy(text[2], "xx,xx,xx,xx,xx,xx,xx,xx");
  strcpy(text[3], "xx    xx xx xx xx    xx");
  text[1][0] = debugSignalToChar(5);
  text[1][6] = debugSignalToChar(3);
  text[0][6] = debugSignalToChar(4);
  text[1][9] = debugSignalToChar(1);
  text[0][9] = debugSignalToChar(2);
  text[1][15] = debugSignalToChar(0);
  text[1][2] = debugRouteToChar(1);
  text[0][5] = debugRouteToChar(2);
  text[1][13] = debugRouteToChar(0);
  text[0][1] = ((debugRouteToChar(1) != '-') ? '*' : '.');
  text[1][3] = debugTurnoutToChar(1, '/');
  text[0][4] = debugTurnoutToChar(2, '/');
  text[1][12] = debugTurnoutToChar(0, '\\');
  for (int t = 0; t < 8; t++)
  {
    int data = dataOut[t + 1];
    text[2][t*3] = num2hex(data >> 4);
    text[2][t*3+1] = num2hex(data);
  }
  signalToText(0, text[3] + 0);
  signalToText(1, text[3] + 6);
  signalToText(2, text[3] + 9);
  signalToText(3, text[3] + 12);
  signalToText(4, text[3] + 15);
  signalToText(5, text[3] + 21);
  for (int t = 0; t < 4; t++)
    Serial.println(text[t]);
}

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

void turnoutSet(int turnoutNum, int route, int routeSelection)
{
  if ((turnoutNum < 0) || (turnoutNum >= TURNOUTS_NUM))
    return;
  Serial.print("setting turnout ");
  Serial.print(turnoutNum);
  Serial.print(" to direction ");
  Serial.println(route);
  turnouts[turnoutNum].state.state = ((routeSelection < 0) ? TURNOUTSTATE_MOVING_TURNOUT : TURNOUTSTATE_MOVING_ROUTE);
  turnouts[turnoutNum].state.counter = TURNOUT_MOVE_TIME;
  turnouts[turnoutNum].state.route = route;
  turnouts[turnoutNum].state.routeSelectionNum = routeSelection;
}

void turnoutStop(int turnoutNum)
{
  if ((turnoutNum < 0) || (turnoutNum >= TURNOUTS_NUM))
    return;
  turnouts[turnoutNum].state.state = TURNOUTSTATE_IDLE;
  turnouts[turnoutNum].state.counter = 0;
}

bool turnoutIsLocked(int turnoutNum)
{
  if ((turnoutNum < 0) || (turnoutNum >= TURNOUTS_NUM))
    return false;
  return (turnouts[turnoutNum].state.state == TURNOUTSTATE_LOCKED);
}

void signalSet(int signalNum, int state)
{
  if ((signalNum < 0) || (signalNum >= SIGNALS_NUM))
    return;
  Serial.print("setting signal ");
  Serial.print(signalNum);
  Serial.print(" to state ");
  Serial.println(state);
  signals[signalNum].state.state = state;
}

bool routeLock(int routeNum, int routeSelectionNum)
{
  if ((routeNum < 0) || (routeNum >= ROUTELOCKS_NUM))
    return false;
  if (routeLocks[routeNum].state.state != ROUTESTATE_IDLE)
    return false;
  Serial.print("setting route lock ");
  Serial.print(routeNum);
  Serial.print(" to route ");
  Serial.println(routeSelectionNum);
  routeLocks[routeNum].state.state = ((routeSelectionNum >= 0) ? ROUTESTATE_ACTIVE : ROUTESTATE_LOCKED);
  routeLocks[routeNum].state.routeSelectionNum = routeSelectionNum;
  return true;
}

bool routeClear(int routeNum)
{
  if ((routeNum < 0) || (routeNum >= ROUTELOCKS_NUM))
    return false;
  if (routeLocks[routeNum].state.state != ROUTESTATE_LOCKED)
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
        switch(turnouts[t].state.state)
        {
          case TURNOUTSTATE_IDLE:
          case TURNOUTSTATE_LOCKED:
            break;
          case TURNOUTSTATE_MOVING_TURNOUT:
            Serial.print("turnout ");
            Serial.print(t);
            Serial.println(" idle");
            turnouts[t].state.state = TURNOUTSTATE_IDLE;
            break;
          case TURNOUTSTATE_MOVING_ROUTE:
            Serial.print("turnout ");
            Serial.print(t);
            Serial.println(" locked");
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

/*
#define SHOW_DARK         (0x00)
#define SHOW_RED          (0x04)
#define SHOW_GREEN        (0x01)
#define SHOW_YELLOW       (0x02)
#define SHOW_GREENYELLOW  (0x09)
#define SHOW_BLANKYELLOW  (0x08)
#define SHOW_YELLOWYELLOW (0x0A)
#define SHOW_CALLING      (0x14)
#define SHOW_TESTALL      (0x1F) 
0  {{{45-LED_FROM,47-LED_FROM,49-LED_FROM},{DRIVER_SIGNAL4,0x01,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
1  {{{42-LED_FROM,44-LED_FROM,46-LED_FROM},{DRIVER_SIGNAL3,0x03,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
2  {{{51-LED_FROM,38-LED_FROM,40-LED_FROM},{DRIVER_SIGNAL4,0x04,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
3  {{{25-LED_FROM,50-LED_FROM,48-LED_FROM},{DRIVER_SIGNAL4,0x05,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
4  {{{31-LED_FROM,29-LED_FROM,27-LED_FROM},{DRIVER_SIGNAL3,0x06,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},
5  {{{28-LED_FROM,26-LED_FROM,24-LED_FROM},{DRIVER_SIGNAL4,0x08,0}},{SIGNAL_STOP,{SHOW_TESTALL,SHOW_DARK}}},

- A fővágányon levő kijárati jelzők zöldek, ha a váltó egyenesben van előttük és van kijárat. Amúgy piros
- A második vágányon levő kijárati jelzők zöld-fekete-fekete-sárgák, ha a váltó kitérőben van (a Füzesabony felőli oldalon a védőváltónak is kitérőben kell állnia). Amúgy piros
- A bejárati jelző zöld, ha a váltó egyenesben van és a kijárati jelző is zöld:
- A bejárati jelző sárga, ha a váltó egyenesben van és a kijárati jelző piros:
- A bejárati jelző dupla sárga, ha a kijárati jelző piros és a váltó kitérőben van:
- A bejárati jelző villogó-sára-sárga, ha a váltó kitérőben van és a kijárati jelzó zöld-sárga:
*/
void updateSignalImages()
{
  for (int t = 0; t < SIGNALS_NUM; t++)
  {
    switch (signals[t].state.state)
    {
      case SIGNAL_STOP:
        signals[t].state.image[0] = SHOW_RED;
        signals[t].state.image[1] = SHOW_RED;
        break;
      case SIGNAL_GO:
        switch (t)
        {
          case 1:
          case 4:
            signals[t].state.image[0] = SHOW_GREEN;
            signals[t].state.image[1] = SHOW_GREEN;
            break;
          case 2:
          case 3:
            signals[t].state.image[0] = SHOW_GREENYELLOW;
            signals[t].state.image[1] = SHOW_GREENYELLOW;
            break;
          case 0:
            if (turnouts[0].state.route == 0)
            {
              if (signals[4].state.state == SIGNAL_GO)
              {
                signals[t].state.image[0] = SHOW_GREEN;
                signals[t].state.image[1] = SHOW_GREEN;
              } else
              {
                signals[t].state.image[0] = SHOW_YELLOW;
                signals[t].state.image[1] = SHOW_YELLOW;
              }
            } else
            {
              if (signals[3].state.state == SIGNAL_GO)
              {
                signals[t].state.image[0] = SHOW_YELLOWYELLOW;
                signals[t].state.image[1] = SHOW_BLANKYELLOW;
              } else
              {
                signals[t].state.image[0] = SHOW_YELLOWYELLOW;
                signals[t].state.image[1] = SHOW_YELLOWYELLOW;
              }
            }
            break;
          case 5:
            if (turnouts[1].state.route == 0)
            {
              if (signals[1].state.state == SIGNAL_GO)
              {
                signals[t].state.image[0] = SHOW_GREEN;
                signals[t].state.image[1] = SHOW_GREEN;
              } else
              {
                signals[t].state.image[0] = SHOW_YELLOW;
                signals[t].state.image[1] = SHOW_YELLOW;
              }
            } else
            {
              if (signals[2].state.state == SIGNAL_GO)
              {
                signals[t].state.image[0] = SHOW_YELLOWYELLOW;
                signals[t].state.image[1] = SHOW_BLANKYELLOW;
              } else
              {
                signals[t].state.image[0] = SHOW_YELLOWYELLOW;
                signals[t].state.image[1] = SHOW_YELLOWYELLOW;
              }
            }
            break;
          default:
            signals[t].state.image[0] = SHOW_GREEN;
            signals[t].state.image[1] = SHOW_DARK;
            break;
        }
        break;
      case SIGNAL_CALL:
        signals[t].state.image[0] = SHOW_CALLING;
        signals[t].state.image[1] = SHOW_RED;
        break;
    }
  }
}

void updateSignals()
{
  for (int t = 0; t < SIGNALS_NUM; t++)
  {
    int address = signals[t].desc.driver.address;
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
    leds[routeLocks[t].desc.led] = ((routeLocks[t].state.state != ROUTESTATE_IDLE) ? LEDS_ON : LEDS_OFF);
    if (routeLocks[t].desc.crossingLed >= 0)
      leds[routeLocks[t].desc.crossingLed] = ((routeLocks[t].state.state != ROUTESTATE_IDLE) ? LEDS_BLINK : LEDS_OFF);
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
      turnoutSet(t, TURNOUT_DIVERGING, -1);
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
      turnoutSet(t, TURNOUT_STRAIGHT, -1);
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
    if (false) // keys[0])
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
  for (int t = 0; t < ROUTELOCKS_NUM; t++)
  {
    routeLocks[t].state.state = ROUTESTATE_IDLE;
  }
}

void runningLogic()
{
  // scan for turnout changes
  for (int t = 0; t < TURNOUTS_NUM; t++)
  {
    if (keys[turnoutSelections[t].changeKeys[0]] && keys[turnoutSelections[t].changeKeys[1]])
    {
      if (turnouts[t].state.state != TURNOUTSTATE_IDLE)
        continue;
      turnoutSet(t, ((turnouts[t].state.route == 0) ? 1 : 0), -1);
    }
  }
  // scan for signal calls
  for (int t = 0; t < SIGNALS_NUM; t++)
  {
    int sn = signalSelections[t].signalNum;
    int r0 = signalSelections[t].routeLockNums[0];
    int r1 = signalSelections[t].routeLockNums[1];
    if (keys[signalSelections[t].callKeys[0]] && keys[signalSelections[t].callKeys[1]])
    {
      if ((r0 != UNUSED) && (routeLocks[r0].state.state != ROUTESTATE_IDLE))
        continue;
      if ((r1 != UNUSED) && (routeLocks[r1].state.state != ROUTESTATE_IDLE))
        continue;
      if (r0 != UNUSED)
        routeLock(r0, -1);
      if (r1 != UNUSED)
        routeLock(r1, -1);
      signalSet(sn, SIGNAL_CALL);
    }
    if (keys[signalSelections[t].clearKeys[0]] && keys[signalSelections[t].clearKeys[1]])
    {
      if (signals[sn].state.state != SIGNAL_CALL)
        continue;
      if (r0 != UNUSED)
        routeClear(r0);
      if (r1 != UNUSED)
        routeClear(r1);
      signalSet(sn, SIGNAL_STOP);
    }
  }
  // scan for route selections
  for (int t = 0; t < ROUTES_NUM; t++)
  {
    if (keys[routeSelections[t].lockKeys[0]] && keys[routeSelections[t].lockKeys[1]])
    {
      int r0 = routeSelections[t].routeLockNums[0];
      int r1 = routeSelections[t].routeLockNums[1];
      int t0 = routeSelections[t].turnoutNums[0];
      int t1 = routeSelections[t].turnoutNums[1];
      int d0 = routeSelections[t].turnoutDirs[0];
      int d1 = routeSelections[t].turnoutDirs[1];
      int br = routeSelections[t].blockLock;
      int bt = routeSelections[t].blockTurnout;
      int bd = routeSelections[t].blockTurnoutDir;
      bool isBlock = false;
      if (br != UNUSED)
        isBlock = ((routeLocks[br].state.state != ROUTESTATE_IDLE) && (turnouts[bt].state.route == bd));
      if (r1 == UNUSED)
      {
        if ((routeLocks[r0].state.state == ROUTESTATE_IDLE) && !isBlock)
        {
          if ((t0 != UNUSED) && (turnouts[t0].state.state != TURNOUTSTATE_IDLE))
            continue;
          routeLock(r0, t);
          if (t0 != UNUSED)
            turnoutSet(t0, d0, t);
        }
      } else
      {
        if ((routeLocks[r0].state.state == ROUTESTATE_IDLE) && (routeLocks[r1].state.state == ROUTESTATE_IDLE) && !isBlock)
        {
          if ((t0 != UNUSED) && (turnouts[t0].state.state != TURNOUTSTATE_IDLE))
            continue;
          if ((t1 != UNUSED) && (turnouts[t1].state.state != TURNOUTSTATE_IDLE))
            continue;
          routeLock(r0, t);
          routeLock(r1, t);
          if (t0 != UNUSED)
            turnoutSet(t0, d0, t);
          if (t1 != UNUSED)
            turnoutSet(t1, d1, t);
        }
      }
    }
  }
  for (int t = 0; t < ROUTELOCKS_NUM; t++)
  {
    if (routeLocks[t].state.state == ROUTESTATE_IDLE)
      continue;
    int rsn = routeLocks[t].state.routeSelectionNum;
    if ((rsn >= 0) && (routeLocks[t].state.state == ROUTESTATE_ACTIVE))
    {
      int t0 = routeSelections[rsn].turnoutNums[0];
      int t1 = routeSelections[rsn].turnoutNums[1];
      bool isDone = true;
      if (t0 >= 0)
        isDone = isDone && turnoutIsLocked(t0);
      if (t1 >= 0)
        isDone = isDone && turnoutIsLocked(t1);
      if (isDone)
      {
        signalSet(routeSelections[rsn].signalNum, SIGNAL_GO);
        routeLocks[t].state.state = ROUTESTATE_LOCKED;
      }
    }
    if (keys[routeLocks[t].desc.unlockKeys[0]] && keys[routeLocks[t].desc.unlockKeys[1]] && (routeLocks[t].state.state == ROUTESTATE_LOCKED))
    {
      if (rsn >= 0)
        signalSet(routeSelections[rsn].signalNum, SIGNAL_STOP);
      routeLocks[t].state.state = ROUTESTATE_IDLE;
      int t0 = routeSelections[rsn].turnoutNums[0];
      int t1 = routeSelections[rsn].turnoutNums[1];
      if (t0 >= 0)
        turnoutStop(t0);
      if (t1 >= 0)
        turnoutStop(t1);
    }
  }
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

// ---------- MAIN ----------

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
      updateRoutes();
      updateSignalImages();
      updateSignals();
      if (modeFlag == MODE_RUN)
      {
        dataOut[2] = (dataOut[2] & 0x0f) | (keys[0] ? 0x10 : 0x00); // station lights on switch 0
        dataOut[7] = (dataOut[7] & 0x0f) | ((routeLocks[1].state.state != ROUTESTATE_IDLE) ? 0x00 : 0x10); // crossing light signal
      }
    }
    sendCommands(false); // sendCommands(dataOutT1 % 10) == 0);
    if ((dataOutT1 % 10) == 0)
      debugImage();
  }
}


