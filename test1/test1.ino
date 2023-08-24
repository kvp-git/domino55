#define LED_FROM (24)
#define LED_TO   (51)

#define KEY_FROM (A0)
#define KEY_TO   (A15)

#define BOARD_LED (13)

int ledNum = LED_FROM;

void setup()
{
  Serial.begin(115200);

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
  Serial.println("Domino 55 - test1");
}

int t0 = HIGH;

void loop()
{
  /*
  digitalWrite(ledNum, HIGH);
  delay(1000);
  digitalWrite(ledNum, LOW);
  ledNum++;
  if (ledNum > LED_TO)
    ledNum = LED_FROM;
  */
  for (int t = KEY_FROM; t <= KEY_TO; t++)
    Serial.print(digitalRead(t), BIN);
  Serial.print("   ");
  int t1 = digitalRead(KEY_FROM);
  if (t0 != t1)
  {
    t0 = t1;
    ledNum++;
    if (ledNum > LED_TO)
      ledNum = LED_FROM;
  }
  Serial.println(ledNum);
  digitalWrite(ledNum, HIGH);
  delay(500);
  digitalWrite(ledNum, LOW);
  delay(500);
}

