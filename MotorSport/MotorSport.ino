#include <LiquidCrystal.h>
#include "SR04.h"

#define GREEN 6
#define RED   3
#define BTN   2

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

bool RUN         = false;
bool FAULT       = false;
bool RUN_UPDATE   = false;
bool FAULT_UPDATE = false;
bool STOP_UPDATE  = false;

// Debounce state
bool     btnLastStable = HIGH;
bool     btnReading    = HIGH;
unsigned long btnLastChange = 0;
const unsigned long DEBOUNCE_MS = 50;

#define TRIG_PIN 5
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

void setup()
{
  pinMode(BTN, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);

  Serial.begin(9600);
  lcd.begin(16, 2);
}

void loop()
{
  // --- Debounced button poll ---
  bool reading = digitalRead(BTN);
  if (reading != btnReading) {
    btnReading    = reading;
    btnLastChange = millis();
  }
  if ((millis() - btnLastChange) >= DEBOUNCE_MS && reading != btnLastStable) {
    btnLastStable = reading;
    if (btnLastStable == LOW) {       // LOW = pressed (INPUT_PULLUP)
      RUN_UPDATE   = false;
      FAULT_UPDATE = false;
      STOP_UPDATE  = false;
      FAULT        = false;
      RUN          = !RUN;
    }
  }

  // --- Main logic ---
  if (RUN)
  {
    // Check for obstacle first
    long Distance = sr04.Distance();
    if (Distance > 0 && Distance < 10)
    {
      RUN          = false;
      FAULT        = true;
      RUN_UPDATE   = false;
      FAULT_UPDATE = false;
      STOP_UPDATE  = false;
    }

    if (FAULT)
    {
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      if (!FAULT_UPDATE)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("FAULT");
        FAULT_UPDATE = true;
        RUN_UPDATE   = false;
      }
      return;
    }

    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);

    if (!RUN_UPDATE)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Running...");
      RUN_UPDATE  = true;
      STOP_UPDATE = false;
    }

    int Pot = analogRead(A0);
    int Vrx = analogRead(A1);
    int Vry = analogRead(A2);

    Serial.print(Pot);
    Serial.print(", ");
    Serial.print(Vrx);
    Serial.print(", ");
    Serial.println(Vry);
  }
  else
  {
    if (FAULT)
    {
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      if (!FAULT_UPDATE)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("FAULT");
        FAULT_UPDATE = true;
      }
    }
    else
    {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      if (!STOP_UPDATE)
      {
        lcd.clear();
        STOP_UPDATE = true;
      }
    }
  }
}