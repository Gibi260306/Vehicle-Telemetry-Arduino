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
enum SystemState
{
  OFF,
  RUNNING,
  WARNING,
  CRASHED,
  SENSOR_FAULT
};

SystemState System_State = OFF;
SystemState Last_Screen_State = OFF;

// Fault flags
const byte NO_FAULT = 0x00;
const byte ULTRASONIC_FAULT = 0x01;
byte Fault_Flags = NO_FAULT;

// Sensor values
float Throttle = 0.0;
float Brake = 0.0;
float Wheel_angle = 0.0;
long Distance = -1;
bool Distance_Valid = false;

// Filtered values
float Filtered_Throttle = 0.0;
float Filtered_Brake = 0.0;
float Filtered_Wheel_angle = 0.0;

// Distance filter
long Distance_Samples[3] = {-1, -1, -1};
int Distance_Sample_Index = 0;
int Valid_Distance_Samples = 0;

// Button interrupt
volatile bool buttonInterruptFlag = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// Update timers
unsigned long lastInputUpdate = 0;
unsigned long lastDistanceUpdate = 0;
unsigned long lastTelemetryUpdate = 0;
unsigned long lastTFTUpdate = 0;
unsigned long lastValidDistanceTime = 0;

const unsigned long INPUT_UPDATE_MS = 10;
const unsigned long DISTANCE_UPDATE_MS = 60;
const unsigned long TELEMETRY_UPDATE_MS = 50;
const unsigned long TFT_UPDATE_MS = 100;
const unsigned long ULTRASONIC_TIMEOUT_MS = 500;

const unsigned long OFF_TELEMETRY_UPDATE_MS = 1000;
const unsigned long CRASH_TELEMETRY_UPDATE_MS = 200;

// ADC calibration values
const int THROTTLE_MIN = 20;
const int THROTTLE_MAX = 1000;
const int BRAKE_MIN = 20;
const int BRAKE_MAX = 1000;

const int STEERING_MIN = 0;
const int STEERING_MAX = 1023;
const int STEERING_CENTRE = 510;
const int STEERING_DEADZONE = 25;
const int MAX_WHEEL_ANGLE = 35;

// Safety distances
const long CRASH_DISTANCE = 10;
const long WARNING_DISTANCE = 20;
const long WARNING_CLEAR_DISTANCE = 23;
const long MAX_DISTANCE = 400;

// Input filter
const float FILTER_ALPHA = 0.20;

// Telemetry packet counter
unsigned long Packet_Number = 0;

// Warning memory for hysteresis
bool Warning_Active = false;

void buttonISR()
{
  buttonInterruptFlag = true;
}

float limitFloat(float Value, float Minimum, float Maximum)
{
  if (Value < Minimum)
  {
    Value = Minimum;
  }

  if (Value > Maximum)
  {
    Value = Maximum;
  }

  return Value;
}

float adcToPercent(int Raw_Value, int Minimum, int Maximum)
{
  if (Maximum <= Minimum)
  {
    return 0.0;
  }

  float Percentage = ((float)(Raw_Value - Minimum) * 100.0) / (float)(Maximum - Minimum);
  Percentage = limitFloat(Percentage, 0.0, 100.0);

  return Percentage;
}

float filterValue(float Old_Value, float New_Value)
{
  return Old_Value + FILTER_ALPHA * (New_Value - Old_Value);
}

long medianOfThree(long A, long B, long C)
{
  long Temp;

  if (A > B)
  {
    Temp = A;
    A = B;
    B = Temp;
  }

  if (B > C)
  {
    Temp = B;
    B = C;
    C = Temp;
  }

  if (A > B)
  {
    Temp = A;
    A = B;
    B = Temp;
  }

  return B;
}

const char* getStateName()
{
  if (System_State == OFF)
  {
    return "OFF";
  }
  else if (System_State == RUNNING)
  {
    return "RUNNING";
  }
  else if (System_State == WARNING)
  {
    return "WARNING";
  }
  else if (System_State == CRASHED)
  {
    return "CRASHED";
  }
  else if (System_State == SENSOR_FAULT)
  {
    return "SENSOR_FAULT";
  }

  return "UNKNOWN";
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

  tft.setCursor(28, 55);
  tft.print("CRASH");

  tft.setCursor(10, 82);
  tft.print("DETECTED");

  tft.setTextSize(1);
  tft.setCursor(14, 115);
  tft.print("Press button to reset");
}

void drawRunScreen()
{
  clearScreen();

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  tft.setCursor(5, 5);
  tft.print("State:");

  tft.setCursor(5, 20);
  tft.print("Throttle:");
  tft.drawRect(5, 31, 70, 7, ST7735_WHITE);

  tft.setCursor(5, 45);
  tft.print("Brake:");
  tft.drawRect(5, 56, 70, 7, ST7735_WHITE);

  tft.setCursor(5, 70);
  tft.print("Wheel Angle:");

  tft.setCursor(5, 85);
  tft.print("Dist:");

  tft.setCursor(5, 105);
  tft.print("Status:");
}

void drawBar(int x, int y, float Percentage)
{
  int Bar_Width = (int)((Percentage / 100.0) * 68.0);

  if (Bar_Width < 0)
  {
    Bar_Width = 0;
  }

  if (Bar_Width > 68)
  {
    Bar_Width = 68;
  }

  tft.drawRect(x, y, 70, 7, ST7735_WHITE);
  tft.fillRect(x + 1, y + 1, 68, 5, ST7735_BLACK);

  if (Bar_Width > 0)
  {
    tft.fillRect(x + 1, y + 1, Bar_Width, 5, ST7735_WHITE);
  }
}

void updateTFT()
{
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

  // Clear only the old number areas
  tft.fillRect(42, 5, 84, 10, ST7735_BLACK);
  tft.fillRect(72, 20, 54, 10, ST7735_BLACK);
  tft.fillRect(72, 45, 54, 10, ST7735_BLACK);
  tft.fillRect(80, 70, 46, 10, ST7735_BLACK);
  tft.fillRect(40, 85, 86, 10, ST7735_BLACK);
  tft.fillRect(5, 116, 121, 10, ST7735_BLACK);

  tft.setCursor(42, 5);
  tft.print(getStateName());

  tft.setCursor(72, 20);
  tft.print(Throttle, 1);
  tft.print("%");
  drawBar(5, 31, Throttle);

  tft.setCursor(72, 45);
  tft.print(Brake, 1);
  tft.print("%");
  drawBar(5, 56, Brake);

  tft.setCursor(80, 70);
  tft.print((int)Wheel_angle);
  tft.print(" deg");

  tft.setCursor(40, 85);

  if (Distance_Valid)
  {
    tft.print(Distance);
    tft.print(" cm");
  }
  else
  {
    tft.print("ERROR");
  }

  tft.setCursor(5, 116);

  if (System_State == WARNING)
  {
    tft.print("Obstacle Imminent");
  }
  else if (System_State == SENSOR_FAULT)
  {
    tft.print("Ultrasonic Fault");
  }
  else
  {
    tft.print("Normal");
  }
}

void updateScreen()
{
  bool Current_Run_Screen =
    System_State == RUNNING ||
    System_State == WARNING ||
    System_State == SENSOR_FAULT;

  bool Last_Run_Screen =
    Last_Screen_State == RUNNING ||
    Last_Screen_State == WARNING ||
    Last_Screen_State == SENSOR_FAULT;

  if (System_State == OFF)
  {
    if (Last_Screen_State != OFF)
    {
      displayOFF();
    }
  }
  else if (System_State == CRASHED)
  {
    if (Last_Screen_State != CRASHED)
    {
      displayCrash();
    }
  }
  else if (Current_Run_Screen)
  {
    // Only draw the complete layout when entering the run screen.
    // WARNING and SENSOR_FAULT use the same layout, so they do not clear it.
    if (!Last_Run_Screen)
    {
      drawRunScreen();
    }

    updateTFT();
  }

  Last_Screen_State = System_State;
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

      if (System_State == OFF)
      {
        System_State = RUNNING;
        lastValidDistanceTime = millis();
      }
      else
      {
        System_State = OFF;
      }

      Warning_Active = false;
      Fault_Flags = NO_FAULT;
      Distance_Valid = false;
      Distance = -1;
      Valid_Distance_Samples = 0;

      Last_Screen_State = (SystemState)255;
    }
  }
}

void readControls()
{
  int Throttle_Raw = analogRead(A0);
  int Brake_Raw = analogRead(A1);
  int Steering_Raw = analogRead(A3);

  float New_Throttle = adcToPercent(Throttle_Raw, THROTTLE_MIN, THROTTLE_MAX);
  float New_Brake = adcToPercent(Brake_Raw, BRAKE_MIN, BRAKE_MAX);
  float New_Wheel_angle = 0.0;

  if (Steering_Raw < STEERING_CENTRE - STEERING_DEADZONE)
  {
    New_Wheel_angle = map(Steering_Raw,
                          STEERING_MIN,
                          STEERING_CENTRE - STEERING_DEADZONE,
                          -MAX_WHEEL_ANGLE,
                          0);
  }
  else if (Steering_Raw > STEERING_CENTRE + STEERING_DEADZONE)
  {
    New_Wheel_angle = map(Steering_Raw,
                          STEERING_CENTRE + STEERING_DEADZONE,
                          STEERING_MAX,
                          0,
                          MAX_WHEEL_ANGLE);
  }

  Filtered_Throttle = filterValue(Filtered_Throttle, New_Throttle);
  Filtered_Brake = filterValue(Filtered_Brake, New_Brake);
  Filtered_Wheel_angle = filterValue(Filtered_Wheel_angle, New_Wheel_angle);

  Throttle = Filtered_Throttle;
  Brake = Filtered_Brake;
  Wheel_angle = limitFloat(Filtered_Wheel_angle, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);
}

void readDistance()
{
  long New_Distance = sr04.Distance();

  if (New_Distance > 0 && New_Distance <= MAX_DISTANCE)
  {
    Distance_Samples[Distance_Sample_Index] = New_Distance;
    Distance_Sample_Index++;

    if (Distance_Sample_Index >= 3)
    {
      Distance_Sample_Index = 0;
    }

    if (Valid_Distance_Samples < 3)
    {
      Valid_Distance_Samples++;
    }

    if (Valid_Distance_Samples == 3)
    {
      Distance = medianOfThree(Distance_Samples[0], Distance_Samples[1], Distance_Samples[2]);
    }
    else
    {
      Distance = New_Distance;
    }

    Distance_Valid = true;
    lastValidDistanceTime = millis();
    Fault_Flags &= ~ULTRASONIC_FAULT;
  }
  else
  {
    if (millis() - lastValidDistanceTime >= ULTRASONIC_TIMEOUT_MS)
    {
      Distance = -1;
      Distance_Valid = false;
      Fault_Flags |= ULTRASONIC_FAULT;
    }
  }
}

void updateState()
{
  if (System_State == OFF || System_State == CRASHED)
  {
    return;
  }

  if (Fault_Flags != NO_FAULT)
  {
    System_State = SENSOR_FAULT;
    return;
  }

  if (!Distance_Valid)
  {
    System_State = RUNNING;
    return;
  }

  if (Distance < CRASH_DISTANCE)
  {
    System_State = CRASHED;
    Warning_Active = false;
    return;
  }

  if (!Warning_Active && Distance < WARNING_DISTANCE)
  {
    Warning_Active = true;
  }
  else if (Warning_Active && Distance > WARNING_CLEAR_DISTANCE)
  {
    Warning_Active = false;
  }

  if (Warning_Active)
  {
    System_State = WARNING;
  }
  else
  {
    System_State = RUNNING;
  }
}

void sendTelemetry()
{
  float Telemetry_Throttle = Throttle;
  float Telemetry_Brake = Brake;
  float Telemetry_Wheel_angle = Wheel_angle;
  long Telemetry_Distance = -1;

  if (Distance_Valid)
  {
    Telemetry_Distance = Distance;
  }

  // When OFF, send a heartbeat but zero the live inputs
  if (System_State == OFF)
  {
    Telemetry_Throttle = 0.0;
    Telemetry_Brake = 0.0;
    Telemetry_Wheel_angle = 0.0;
    Telemetry_Distance = -1;
  }

  Serial.print("VTP1,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(Packet_Number);
  Serial.print(",");
  Serial.print(Telemetry_Throttle, 1);
  Serial.print(",");
  Serial.print(Telemetry_Brake, 1);
  Serial.print(",");
  Serial.print((int)Telemetry_Wheel_angle);
  Serial.print(",");
  Serial.print(Telemetry_Distance);
  Serial.print(",");
  Serial.print(getStateName());
  Serial.print(",");
  Serial.println(Fault_Flags);

  Packet_Number++;
}

void setup()
{
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, FALLING);

  Serial.begin(115200);

  tft.initR(INITR_BLACKTAB);
  tft.invertDisplay(false);
  tft.setRotation(0);

  displayOFF();
  Last_Screen_State = OFF;
  lastButtonPressTime = millis() - BUTTON_DEBOUNCE_MS;
}

void loop()
{
  handleButtonPress();

  if (System_State != OFF && System_State != CRASHED)
  {
    if (millis() - lastInputUpdate >= INPUT_UPDATE_MS)
    {
      lastInputUpdate = millis();
      readControls();
    }

    if (millis() - lastDistanceUpdate >= DISTANCE_UPDATE_MS)
    {
      lastDistanceUpdate = millis();
      readDistance();
    }
  }

  updateState();

  unsigned long Telemetry_Interval = TELEMETRY_UPDATE_MS;

  if (System_State == OFF)
  {
    Telemetry_Interval = OFF_TELEMETRY_UPDATE_MS;
  }
  else if (System_State == CRASHED)
  {
    Telemetry_Interval = CRASH_TELEMETRY_UPDATE_MS;
  }

  if (millis() - lastTelemetryUpdate >= Telemetry_Interval)
  {
    lastTelemetryUpdate = millis();
    sendTelemetry();
  }

  // This block was missing, which left the TFT showing OFF
  // even after System_State changed to RUNNING.
  if (millis() - lastTFTUpdate >= TFT_UPDATE_MS)
  {
    lastTFTUpdate = millis();
    updateScreen();
  }
}