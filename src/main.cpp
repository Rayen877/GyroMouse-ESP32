#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define LEFT_BUTTON  23  
#define RIGHT_BUTTON 32   

#define DEADZONE     0.14f
#define SENSITIVITY  18
#define DEBOUNCE_DELAY 50  // ms

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32-MPU-Mouse", "ESP32", 100);

bool lastLeft  = HIGH;
bool lastRight = HIGH;

unsigned long lastLeftTime  = 0;
unsigned long lastRightTime = 0;

float alpha = 0.1;  // Low pass filter factor
float rollFiltered  = 0;
float pitchFiltered = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 not found!");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 ready");

  bleMouse.begin();
  Serial.println("BLE Mouse started");
}

void loop() {
  if (!bleMouse.isConnected()) {
    delay(50);
    return;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Read raw roll and pitch
  float roll  = accel.acceleration.x / 9.81f;
  float pitch = accel.acceleration.y / 9.81f;

  // Apply low pass filter
  rollFiltered  = alpha * roll  + (1 - alpha) * rollFiltered;
  pitchFiltered = alpha * pitch + (1 - alpha) * pitchFiltered;

  int xMove = 0;
  int yMove = 0;

  if (abs(rollFiltered) > DEADZONE)
    xMove = rollFiltered * SENSITIVITY;    

  if (abs(pitchFiltered) > DEADZONE)
    yMove = -pitchFiltered * SENSITIVITY;  

  if (xMove || yMove)
    bleMouse.move(xMove, yMove);

  bool leftNow  = digitalRead(LEFT_BUTTON);
  bool rightNow = digitalRead(RIGHT_BUTTON);
  unsigned long now = millis();

  // LEFT BUTTON
  if (leftNow != lastLeft && (now - lastLeftTime) > DEBOUNCE_DELAY) {
    if (leftNow == LOW)
      bleMouse.press(MOUSE_LEFT);
    else
      bleMouse.release(MOUSE_LEFT);
    lastLeftTime = now;
  }

  // RIGHT BUTTON
  if (rightNow != lastRight && (now - lastRightTime) > DEBOUNCE_DELAY) {
    if (rightNow == LOW)
      bleMouse.press(MOUSE_RIGHT);
    else
      bleMouse.release(MOUSE_RIGHT);
    lastRightTime = now;
  }

  lastLeft  = leftNow;
  lastRight = rightNow;

  delay(10);
}
