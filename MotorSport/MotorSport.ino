#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include "SR04.h"

#define BTN   2

#define TFT_CS  7
#define TFT_RST 5
#define TFT_DC  6

bool POT_FAULT   = false;
bool STICK_FAULT = false;
bool SONIC_FAULT = false;

bool RUN         = false;
bool FAULT       = false;
bool RUN_UPDATE   = false;
bool FAULT_UPDATE = false;
bool STOP_UPDATE  = false;

// Debounce state
bool          btnLastStable = HIGH;
bool          btnReading    = HIGH;
unsigned long btnLastChange = 0;
const unsigned long DEBOUNCE_MS = 50;
unsigned long Timer = 0;

#define TRIG_PIN 4
#define ECHO_PIN 3
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const float MAX_SPEED   = 100.0;
const float MIN_SPEED   = 0.0;
const float ACCELERATION = 5.0;
const float DECELERATION = 8.0;
static float Car_Speed  = 0.0;

void tftShowFault()
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(10, 55);
  tft.print("FAULT");
}

void tftShowRunning()
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(10, 10);
  tft.print("Running...");
}

void tftShowSpeed(float speed)
{
  tft.fillRect(0, 30, 128, 20, ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 30);
  tft.print((int)speed);
  tft.print(" km/h");
}

void tftShowStopped()
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 55);
  tft.print("Stopped");
}

void setup()
{
  pinMode(BTN, INPUT_PULLUP);

  // Manual hardware reset before init — fixes random pixels on most ST7735 clones
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  Serial.begin(9600);
  Timer = millis();

  // Try these init types in order if display still shows garbage:
  // INITR_BLACKTAB, INITR_GREENTAB, INITR_144GREENTAB, INITR_GREENTAB128
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tftShowStopped();
  btnLastChange = millis();
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
    if (btnLastStable == LOW) {
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
      if (!FAULT_UPDATE)
      {
        tftShowFault();
        FAULT_UPDATE = true;
        RUN_UPDATE   = false;
      }
      return;
    }

    if (!RUN_UPDATE)
    {
      tftShowRunning();
      RUN_UPDATE  = true;
      STOP_UPDATE = false;
    }

    // Read inputs
    int Pot = analogRead(A0);
    int Vrx = analogRead(A1);
    int Vry = analogRead(A2);

    // Check pot fault
    if (Pot < 0 || Pot > 1023)
    {
      POT_FAULT = true;
    }

    // Calculate target speed
    float target_speed = (float)map(Pot, 0, 1023, MIN_SPEED, MAX_SPEED);

    // Update car speed smoothly
    if (Car_Speed != target_speed)
    {
      unsigned long current_time = millis();
      unsigned long dif = current_time - Timer;
      Timer = current_time;

      if (dif >= 50)
      {
        float dt_sec = dif / 1000.0;

        if (Car_Speed < target_speed)
        {
          Car_Speed += ACCELERATION * dt_sec;
          if (Car_Speed > target_speed) Car_Speed = target_speed;
        }
        else if (Car_Speed > target_speed)
        {
          Car_Speed -= DECELERATION * dt_sec;
          if (Car_Speed < target_speed) Car_Speed = target_speed;
        }
        tftShowSpeed(Car_Speed);
      }
    }

    Serial.print("Pot: ");
    Serial.print(Pot);
    Serial.print(" | Target: ");
    Serial.print(target_speed);
    Serial.print(" | Speed: ");
    Serial.print(Car_Speed);
    Serial.print(" | Vrx: ");
    Serial.print(Vrx);
    Serial.print(" | Vry: ");
    Serial.println(Vry);
  }
  else
  {
    if (FAULT)
    {
      if (!FAULT_UPDATE)
      {
        tftShowFault();
        FAULT_UPDATE = true;
      }
    }
    else
    {
      if (!STOP_UPDATE)
      {
        tftShowStopped();
        STOP_UPDATE = true;
      }
    }
  }
}
