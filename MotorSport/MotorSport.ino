#include "SR04.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define BTN 2

#define TFT_CS   7
#define TFT_DC   8
#define TFT_RST  5

#define TRIG_PIN 4
#define ECHO_PIN 3

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// System states
bool RUN   = false;
bool CRASH = false;

// Screen update flags
bool RUN_UPDATE   = false;
bool STOP_UPDATE  = false;
bool CRASH_UPDATE = false;

// Button interrupt
volatile bool buttonInterruptFlag = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// TFT update timer
unsigned long lastTFTUpdate = 0;
const unsigned long TFT_UPDATE_MS = 100;

// Speed timing
unsigned long Timer = 0;

// Speed values
const float MAX_SPEED    = 100.0;
const float MIN_SPEED    = 0.0;
const float ACCELERATION = 5.0;
const float DECELERATION = 8.0;
static float target_speed = 0.0;
static float Car_Speed   = 0.0;

void buttonISR()
{
  buttonInterruptFlag = true;
}

void clearScreen()
{
  tft.fillScreen(ST7735_BLACK);
}

void displayCenteredText(const char* text, int textSize)
{
  int16_t x1, y1;
  uint16_t w, h;

  tft.setTextSize(textSize);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int x = (tft.width() - w) / 2 - x1;
  int y = (tft.height() - h) / 2 - y1;

  tft.setCursor(x, y);
  tft.print(text);
}

void displayOFF()
{
  clearScreen();
  displayCenteredText("OFF", 2);
}

void displayCrash()
{
  clearScreen();

  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  tft.setCursor(34, 58);
  tft.print("Crash");

  tft.setCursor(16, 84);
  tft.print("Detected");
}

void drawRunScreen()
{
  tft.fillScreen(ST7735_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  tft.setCursor(5, 5);
  tft.print("Pot:");

  tft.setCursor(5, 25);
  tft.print("Target:");

  // Bar between Target and Speed
  tft.drawRect(5, 36, 70, 7, ST7735_WHITE);

  tft.setCursor(5, 45);
  tft.print("Speed:");

  tft.setCursor(5, 65);
  tft.print("Wheel Angle:");

  tft.setCursor(5, 85);
  tft.print("Dist:");
}

void updateTFT(int Pot, float target_speed, float Car_Speed, int Wheel_angle, long Distance)
{
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  // Clear only the old number areas
  tft.fillRect(60, 5,  65, 10, ST7735_BLACK);   // Pot
  tft.fillRect(60, 25, 65, 10, ST7735_BLACK);   // Target
  tft.fillRect(60, 45, 65, 10, ST7735_BLACK);   // Speed
  tft.fillRect(80, 65, 45, 10, ST7735_BLACK);   // Wheel angle
  tft.fillRect(60, 85, 65, 10, ST7735_BLACK);   // Distance
  tft.fillRect(5, 105, 120, 10, ST7735_BLACK);   // Distance

  // Redraw only the numbers
  tft.setCursor(60, 5);
  tft.print(Pot);

  tft.setCursor(60, 25);
  tft.print(target_speed, 1);

  // Throttle bar between Target and Speed
  int barWidth = (int)((target_speed / 100.0) * 68.0);

  if (barWidth < 0) barWidth = 0;
  if (barWidth > 68) barWidth = 68;

  tft.drawRect(5, 36, 70, 7, ST7735_WHITE);
  tft.fillRect(6, 37, 68, 5, ST7735_BLACK);
  tft.fillRect(6, 37, barWidth, 5, ST7735_WHITE);

  tft.setCursor(60, 45);
  tft.print(Car_Speed, 1);

  tft.setCursor(80, 65);
  tft.print(Wheel_angle);
  tft.print(" deg");

  tft.setCursor(60, 85);
  tft.print(Distance);

  if(Distance < 20)
  {
    tft.setCursor(5,  105);
    tft.print("Obstacle Imminent");
  }
}
void handleButtonPress()
{
  bool buttonPressed = false;

  noInterrupts();
  if (buttonInterruptFlag)
  {
    buttonInterruptFlag = false;
    buttonPressed = true;
  }
  interrupts();

  if (buttonPressed)
  {
    if (millis() - lastButtonPressTime >= BUTTON_DEBOUNCE_MS)
    {
      lastButtonPressTime = millis();

      RUN = !RUN;

      CRASH = false;

      RUN_UPDATE   = false;
      STOP_UPDATE  = false;
      CRASH_UPDATE = false;

      if (!RUN)
      {
        Car_Speed = 0.0;
      }
    }
  }
}

void setup()
{
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, FALLING);

  Serial.begin(9600);

  Timer = millis();

  tft.initR(INITR_BLACKTAB);
  tft.invertDisplay(false);
  tft.setRotation(0);

  displayOFF();
  STOP_UPDATE = true;
}

void loop()
{
  handleButtonPress();

  if (RUN)
  {
    if(!RUN_UPDATE)
    {
      drawRunScreen();
      RUN_UPDATE = true;
    }
    long Distance = sr04.Distance();
    if (Distance > 0 && Distance < 10)
    {
      RUN = false;
      CRASH = true;
      Car_Speed = 0.0;

      RUN_UPDATE   = false;
      STOP_UPDATE  = false;

      if (!CRASH_UPDATE)
      {
        displayCrash();
        CRASH_UPDATE = true;
      }

      return;
    }

    int Pot = analogRead(A0);
    int Vrx = analogRead(A1);
    int Vry = analogRead(A2);

    if(Pot < 50)
    {
      target_speed = MIN_SPEED;
    }
    else if(Pot > 973)
    {
      target_speed = MAX_SPEED;
    }
    else
    {
      target_speed = (float)map(Pot, 50, 973, MIN_SPEED, MAX_SPEED);
    } 

    if (Car_Speed != target_speed)
    {
      unsigned long current_time = millis();
      unsigned long dif = current_time - Timer;

      if (dif >= 50)
      {
        Timer = current_time;

        float dt_sec = dif / 1000.0;

        if (Car_Speed < target_speed)
        {
          Car_Speed += ACCELERATION * dt_sec;

          if (Car_Speed > target_speed)
          {
            Car_Speed = target_speed;
          }
        }
        else if (Car_Speed > target_speed)
        {
          Car_Speed -= DECELERATION * dt_sec;

          if (Car_Speed < target_speed)
          {
            Car_Speed = target_speed;
          }
        }
      }
    }

    int Wheel_angle;

    if (Vry > 485 && Vry < 535)
    {
      Wheel_angle = 0;
    }
    else
    {
      Wheel_angle = map(Vry, 0, 1023, -35, 35);
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

    if (millis() - lastTFTUpdate >= TFT_UPDATE_MS)
    {
      lastTFTUpdate = millis();
      updateTFT(Pot, target_speed, Car_Speed, Wheel_angle , Distance);
    }
  }
  else
  {
    if (CRASH)
    {
      if (!CRASH_UPDATE)
      {
        displayCrash();
        CRASH_UPDATE = true;
      }

      return;
    }

    if (!STOP_UPDATE)
    {
      displayOFF();
      STOP_UPDATE = true;
    }
  }
}