#define PIN_IN1    (46)
#define PIN_OUT1   (47)
#define PIN_TO1    (49)
#define PIN_TO2    (51)
#define PIN_IN2    (52)
#define PIN_OUT2   (53)
#define PIN_LIGHTS (48)
// unused connector: 50

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(PIN_IN1,    INPUT_PULLUP);
  pinMode(PIN_OUT1,   INPUT_PULLUP);
  pinMode(PIN_TO1,    INPUT_PULLUP);
  pinMode(PIN_TO2,    INPUT_PULLUP);
  pinMode(PIN_IN2,    INPUT_PULLUP);
  pinMode(PIN_OUT2,   INPUT_PULLUP);
  pinMode(PIN_LIGHTS, INPUT_PULLUP);
  delay(500);
}

void send_cmd(uint8_t addr, uint8_t data)
{
  uint8_t msg[3];
  msg[0] = (addr & 127) | 128;
  msg[1] = data & 127;
  msg[2] = (-(msg[0] + msg[1])) & 127;
  Serial.println(addr, HEX);
  Serial.println(data, HEX);
  Serial.println("");
  Serial1.write(msg, sizeof(msg)); 
}

bool stIn1 = false;
bool stOut1 = false;
bool stIn2 = false;
bool stOut2 = false;
bool stTo1 = false;
bool stTo2 = false;
bool stLights = false;
bool stTo1o = false;
bool stTo2o = false; 

#define SHOW_RED (0x04)
#define SHOW_GREEN (0x01)
#define SHOW_YELLOW (0x02)
#define SHOW_GREENYELLOW (0x09)
#define SHOW_YELLOWYELLOW (0x0a)
#define SHOW_CALLING (0x11)

uint8_t headIn1 = SHOW_RED;
uint8_t headOut1a = SHOW_RED;
uint8_t headOut1b = SHOW_RED;
uint8_t headIn2 = SHOW_RED;
uint8_t headOut2a = SHOW_RED;
uint8_t headOut2b = SHOW_RED;

void loop()
{
  stTo1o = stTo1;
  stTo2o = stTo2;
  stIn1 = (digitalRead(PIN_IN1) == LOW);
  stIn2 = (digitalRead(PIN_IN2) == LOW);
  stOut1 = (digitalRead(PIN_OUT1) == LOW);
  stOut2 = (digitalRead(PIN_OUT2) == LOW);
  if(!stIn1 && !stOut1) stTo1 = (digitalRead(PIN_TO1) == LOW);
  if(!stIn2 && !stOut2) stTo2 = (digitalRead(PIN_TO2) == LOW);
  stLights = (digitalRead(PIN_LIGHTS) == HIGH);

  if(stIn1 && stIn2 && (stTo1 == stTo2))
  {
    stIn1 = false;
    stIn2 = false;
  }
  if(stOut1)
  {
    if(stTo1)
      headOut1a = SHOW_GREEN; 
    else
      headOut1b = SHOW_GREENYELLOW;
  }else
  {
    headOut1a = SHOW_RED;
    headOut1b = SHOW_RED;
  }
  if(stOut2)
  {
    if(stTo2)
      headOut2a = SHOW_GREEN; 
    else
      headOut2b = SHOW_GREENYELLOW;
  }else
  {
    headOut2a = SHOW_RED;
    headOut2b = SHOW_RED;
  }
  if(stIn1)
  {
    if(stTo1)
    {
      if(headOut2a == SHOW_RED)
        headIn1 = SHOW_YELLOW;
      else
        headIn1 = SHOW_GREEN; 
    }else
    {
      if(headOut2b == SHOW_RED)
        headIn1 = SHOW_YELLOWYELLOW;
      else
        headIn1 = SHOW_GREENYELLOW;
    }
  }else
  {
    headIn1 = SHOW_RED;
  }
  if(stIn2)
  {
    if(stTo2)
    {
      if(headOut1a == SHOW_RED)
        headIn2 = SHOW_YELLOW;
      else
        headIn2 = SHOW_GREEN; 
    }else
    {
      if(headOut1b == SHOW_RED)
        headIn2 = SHOW_YELLOWYELLOW;
      else
        headIn2 = SHOW_GREENYELLOW;
    }
  }else
  {
    headIn2 = SHOW_RED;
  }
  if((stTo1 != stTo1o) || (stTo2 != stTo2o))
  {
    if(stTo1 != stTo1o)
    {
      send_cmd(0x01, SHOW_RED);
      send_cmd(0x03, SHOW_RED);
      send_cmd(0x04, SHOW_RED);
    }
    if(stTo2 != stTo2o)
    {
      send_cmd(0x06, SHOW_RED);
      send_cmd(0x05, SHOW_RED);
      send_cmd(0x08, SHOW_RED);
    }
    for(uint8_t t = 0; t < 30; t++)
    {
      send_cmd(0x02, (stTo1 ? 0x02 : 0x01) | (stLights ? 0x10 : 0x00));
      send_cmd(0x07, (stTo2 ? (0x01 | 0x08) : (0x02 | 0x04)) | ((!stIn2 && !stOut2) ? 0x10 : 0x00));
      delay(100);
    }
  }
  send_cmd(0x02, (stTo1 ? 0x02 : 0x01) | (stLights ? 0x10 : 0x00));  
  send_cmd(0x07, (stTo2 ? (0x01 | 0x08) : (0x02 | 0x04)) | ((!stIn2 && !stOut2) ? 0x10 : 0x00));
  send_cmd(0x01, headIn1);
  send_cmd(0x03, headOut1a);
  send_cmd(0x04, headOut1b);
  send_cmd(0x06, headOut2a);
  send_cmd(0x05, headOut2b);
  send_cmd(0x08, headIn2);
}

