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
  tft.print("Throttle:");

  // Bar for Throttle
  tft.drawRect(5, 16, 70, 7, ST7735_WHITE);

  tft.setCursor(5, 30);
  tft.print("Brake:");

  // Bar for Brake
  tft.drawRect(5, 41, 70, 7, ST7735_WHITE);

  tft.setCursor(5, 55);
  tft.print("Wheel Angle:");

  tft.setCursor(5, 75);
  tft.print("Dist:");
}

void updateTFT(int Throttle, int Brake, int Wheel_angle, long Distance)
{
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  // Clear only the old number areas
  tft.fillRect(70, 5,  55, 10, ST7735_BLACK);   // Throttle value
  tft.fillRect(70, 30, 55, 10, ST7735_BLACK);   // Brake value
  tft.fillRect(80, 55, 45, 10, ST7735_BLACK);   // Wheel angle
  tft.fillRect(60, 75, 65, 10, ST7735_BLACK);   // Distance
  tft.fillRect(5, 95, 120, 10, ST7735_BLACK);   // Obstacle warning

  // Redraw only the numbers
  tft.setCursor(70, 5);
  tft.print(Throttle);

  // Throttle bar (0-1023 raw ADC range mapped to bar width)
  int throttleBarWidth = map(Throttle, 0, 1023, 0, 68);
  if (throttleBarWidth < 0) throttleBarWidth = 0;
  if (throttleBarWidth > 68) throttleBarWidth = 68;

  tft.drawRect(5, 16, 70, 7, ST7735_WHITE);
  tft.fillRect(6, 17, 68, 5, ST7735_BLACK);
  tft.fillRect(6, 17, throttleBarWidth, 5, ST7735_WHITE);

  tft.setCursor(70, 30);
  tft.print(Brake);

  // Brake bar (0-1023 raw ADC range mapped to bar width)
  int brakeBarWidth = map(Brake, 0, 1023, 0, 68);
  if (brakeBarWidth < 0) brakeBarWidth = 0;
  if (brakeBarWidth > 68) brakeBarWidth = 68;

  tft.drawRect(5, 41, 70, 7, ST7735_WHITE);
  tft.fillRect(6, 42, 68, 5, ST7735_BLACK);
  tft.fillRect(6, 42, brakeBarWidth, 5, ST7735_WHITE);

  tft.setCursor(80, 55);
  tft.print(Wheel_angle);
  tft.print(" deg");

  tft.setCursor(60, 75);
  tft.print(Distance);

  if(Distance < 20)
  {
    tft.setCursor(5,  95);
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
  
    int Throttle = analogRead(A0);
    int Brake = analogRead(A1);
    int Vrx = analogRead(A2);
    int Vry = analogRead(A3);

    int Wheel_angle;

    if (Vry > 485 && Vry < 535)
    {
      Wheel_angle = 0;
    }
    else
    {
      Wheel_angle = map(Vry, 0, 1023, -35, 35);
    }

    Serial.print(Throttle);
    Serial.print(" , ");
    Serial.print(Brake);
    Serial.print(" , ");
    Serial.println(Wheel_angle);

    if (millis() - lastTFTUpdate >= TFT_UPDATE_MS)
    {
      lastTFTUpdate = millis();
      updateTFT(Throttle, Brake, Wheel_angle, Distance);
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
