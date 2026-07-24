#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "LedControl.h"

// ------------------------------------
int MAX_DISTANCE = 100;
int MIN_DISTANCE = 25;
int INTENSITY = 5;
int TIME_DURATION_ACCELERATE = 1500;
// ------------------------------------

const int TRIG_PIN = 6;
const int ECHO_PIN = 5;
const int DIN_PIN  = 8;
const int CS_PIN   = 9;
const int CLK_PIN  = 10;
const int BUZZER   = 7;

// Function Prototypes (Required for PlatformIO)
float getDistance();
void handleBlink(unsigned long currentMillis, unsigned long interval);
void turnOn();
void turnOff();

LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
Adafruit_MPU6050 mpu;

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 30;

unsigned long lastToggleTime = 0;
bool stateOn = false;
float distance = 999.0;

unsigned long accelOverrideStart = 0;
bool isAccelAlert = false;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);

  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  Wire.begin();
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = currentMillis;
    distance = getDistance();

    Serial.print("Distance: ");
    if (distance > 400.0 || distance <= 0.0) {
      Serial.println("Out of Range");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                                a.acceleration.y * a.acceleration.y + 
                                a.acceleration.z * a.acceleration.z);

    float threshold = 9.81 + (10.0 - INTENSITY) * 1.5;

    if (accelMagnitude > threshold) {
      isAccelAlert = true;
      accelOverrideStart = currentMillis;
    }
  }

  if (isAccelAlert) {
    if (currentMillis - accelOverrideStart < (unsigned long)TIME_DURATION_ACCELERATE) {
      handleBlink(currentMillis, 50);
    } else {
      isAccelAlert = false;
      turnOff();
    }
  } 
  else if (distance > 0.0 && distance <= MIN_DISTANCE) {
    turnOn();
  }
  else if (distance > MIN_DISTANCE && distance <= MAX_DISTANCE) {
    unsigned long interval = map(distance, MIN_DISTANCE, MAX_DISTANCE, 10, 500);
    interval = constrain(interval, 10, 500);
    handleBlink(currentMillis, interval);
  } 
  else {
    turnOff();
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 999.0;

  return (duration * 0.0343) / 2.0;
}

void handleBlink(unsigned long currentMillis, unsigned long interval) {
  if (currentMillis - lastToggleTime >= interval) {
    lastToggleTime = currentMillis;
    stateOn = !stateOn;

    if (stateOn) {
      turnOn();
    } else {
      turnOff();
    }
  }
}

void turnOn() {
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, B11111111);
  }
  digitalWrite(BUZZER, HIGH);
}

void turnOff() {
  lc.clearDisplay(0);
  digitalWrite(BUZZER, LOW);
}