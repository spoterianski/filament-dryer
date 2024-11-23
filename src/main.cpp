#include <Arduino.h>
#include <TM1637Display.h>
#include <PID_v1.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define CLK 2
#define DIO 3
#define HEATER_PIN 11
#define SET_MODE_TIME 50
#define DISPLAY_MODE_TIME 40
#define DHTTYPE    DHT22
#define DHTPIN 4
#define BUTTON_PLUS_PIN 7
#define BUTTON_MINUS_PIN 8
#define MIN_TEMP 0
#define MAX_TEMP 80
#define DEFAULT_TEMP 25
#define LOOP_DELAY 100

typedef enum {
    DISPLAY_MODE_PID = 0,
    DISPLAY_MODE_TEMP,
    DISPLAY_MODE_HUMIDITY
} DisplayMode;

// buttons
int buttonStatePlus = 0;
int buttonStateMinus = 0;
// Sensors
int currentTemperature;
int currentHumidity;
double input, output;
double targetTemp = 0;
int setMode = SET_MODE_TIME;
int displayMode = DISPLAY_MODE_PID;
int displayModeTime = DISPLAY_MODE_TIME;
unsigned long previousMillis = 0;
uint32_t delayMS;
// PID
double Kp = 2.0, Ki = 0.5, Kd = 0.25;

/* Display

    -- A --
   |       |
   F       B
   |       |
    -- G --
   |       |
   E       C
   |       |
    -- D --  (и дополнительная точка DP)

*/
const uint8_t SEG_P[] = {
    SEG_A | SEG_B | SEG_G | SEG_F | SEG_E // P
    };

const uint8_t SEG_T[] = {
    SEG_F | SEG_E | SEG_G | SEG_D, 0x00// t
    };

const uint8_t SEG_H[] = {
    SEG_F | SEG_E | SEG_G | SEG_B | SEG_C, 0x00 // H
    };

const uint8_t SEG_S[] = {
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, // S
    0x00};

TM1637Display display(CLK, DIO);
DHT_Unified dht(DHTPIN, DHTTYPE);
PID myPID(&input, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PLUS_PIN, INPUT);
  pinMode(BUTTON_MINUS_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  display.setBrightness(0x0f);
  targetTemp = DEFAULT_TEMP;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  Serial.print  (F("Min delay:  ")); Serial.println(sensor.min_delay);
}


void loop() {
  unsigned long currentMillis = millis();

  buttonStatePlus = digitalRead(BUTTON_PLUS_PIN);
  buttonStateMinus = digitalRead(BUTTON_MINUS_PIN);
  // Button Plus pushed
  if (buttonStatePlus == HIGH)
  {
    setMode = SET_MODE_TIME;
    targetTemp++;
    Serial.println("Pushed Plus button");
    if (targetTemp >= MAX_TEMP)
    {
      targetTemp = MAX_TEMP;
    }
  }

  // Button Minus pushed
  if (buttonStateMinus == HIGH)
  {
    setMode = SET_MODE_TIME;
    Serial.println("Pushed Minus button");
    targetTemp--;
    if (targetTemp < MIN_TEMP)
    {
      targetTemp = MIN_TEMP;
    }
  }

  if (currentMillis - previousMillis >= delayMS) {
    // Read temperature and humidity
    sensors_event_t event;
    previousMillis = currentMillis;
    // read the sensor
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) {
      currentTemperature = event.temperature;
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) {
      currentHumidity = event.relative_humidity;
    }
  }

  if(setMode == 0){
    // Calculate PID
    input = currentTemperature;
    myPID.Compute();
    // Control the heater
    analogWrite(HEATER_PIN, (int)output);
    // Show parameters on display
    switch (displayMode)
    {
      case DISPLAY_MODE_PID:
        // Show PID output
        display.setSegments(SEG_P, 1, 0);
        display.showNumberDec((int)output, true, 3, 1);
        break;
      case DISPLAY_MODE_TEMP:
        // Show current temperature
        display.setSegments(SEG_T, 2, 0);
        display.showNumberDec(currentTemperature, true, 2, 2);
        break;
      case DISPLAY_MODE_HUMIDITY:
        // Show current humidity
        display.setSegments(SEG_H, 2, 0);
        display.showNumberDec(currentHumidity, true, 2, 2);
        break;
      default:
        break;
    }
    displayModeTime--;
    if(displayModeTime == 0){
      // Change display mode
      displayMode++;
      if(displayMode > DISPLAY_MODE_HUMIDITY){
        displayMode = DISPLAY_MODE_PID;
      }
      displayModeTime = DISPLAY_MODE_TIME;
    }
    Serial.print("pid: ");
    Serial.println(output);
    Serial.print("temp: ");
    Serial.println(currentTemperature);
    Serial.print("himidity: ");
    Serial.println(currentHumidity);
  }
  else{
    // Show the SET mode and selected temperature
    display.setSegments(SEG_S, 2, 0);
    display.showNumberDec(targetTemp, true, 2, 2);
    Serial.print("targetTemp: ");
    Serial.println(targetTemp);
    setMode --;
  }
  delay(LOOP_DELAY);
}