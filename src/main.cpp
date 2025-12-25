#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32-MPU-Mouse", "ESP32", 100);

#define SDA_PIN 21
#define SCL_PIN 22

#define LEFT_BUTTON  23  
#define RIGHT_BUTTON 32   

#define DEADZONE     3.5f
#define DEBOUNCE_DELAY 50  // ms
#define MAX_SPEED 10  

#define MOUSE_SMOOTH 0.2f  

float pitch = 0.0;
float roll  = 0.0;

float SENSITIVITY = 0.65f;
float dxFiltered = 0.0f;
float dyFiltered = 0.0f;
float rollFiltered  = 0;
float pitchFiltered = 0;
   
unsigned long lastTime = 0;

bool lastLeft  = HIGH;
bool lastRight = HIGH;
unsigned long lastLeftTime  = 0;
unsigned long lastRightTime = 0;

// complementary filter coefficient
float alpha = 0.98f;


void moveMouseFromAngles(float pitch, float roll) {
  int dx = 0;
  int dy = 0;

 // left /right
  if (abs(roll) > DEADZONE) {
    dx = (roll > 0) ? (roll - DEADZONE) * SENSITIVITY : (roll + DEADZONE) * SENSITIVITY;
  }
 // up /down
  if (abs(pitch) > DEADZONE) {
    dy = (pitch > 0) ? -(pitch - DEADZONE) * SENSITIVITY : -(pitch + DEADZONE) * SENSITIVITY;
  }
  dxFiltered = MOUSE_SMOOTH * dx + (1.0f - MOUSE_SMOOTH) * dxFiltered;
  dyFiltered = MOUSE_SMOOTH * dy + (1.0f - MOUSE_SMOOTH) * dyFiltered;
  
  // to reduce cursor accel
  int dxOut = constrain((int)dxFiltered, -MAX_SPEED, MAX_SPEED);
  int dyOut = constrain((int)dyFiltered, -MAX_SPEED, MAX_SPEED);

  if (dx || dy) {
    bleMouse.move(dxOut, dyOut);
  }
}

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
  lastTime = micros();
}

void loop() {
  if (!bleMouse.isConnected()) {
    delay(50);
    return;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;

  float pitchAcc = atan2(accel.acceleration.y, accel.acceleration.z) * ( 180.0f/3.1415f) ; // to convert rad to degres
  float rollAcc  = atan2(-accel.acceleration.x,accel.acceleration.z) *( 180.0f/3.1415f) ;
  pitch += gyro.gyro.x * ( 180.0f/3.1415f) * dt;
  roll  += gyro.gyro.y * ( 180.0f/3.1415f) * dt;

  pitch = alpha * pitch + (1 - alpha) * pitchAcc;
  roll  = alpha * roll  + (1 - alpha) * rollAcc;
 
  moveMouseFromAngles(pitch, roll);

  bool leftNow  = digitalRead(LEFT_BUTTON);
  bool rightNow = digitalRead(RIGHT_BUTTON);
  unsigned long now1 = millis();

  // LEFT BUTTON
  if (leftNow != lastLeft && (now1 - lastLeftTime) > DEBOUNCE_DELAY) {
    if (leftNow == LOW)
      bleMouse.press(MOUSE_LEFT);
    else
      bleMouse.release(MOUSE_LEFT);
    lastLeftTime = now1;
  }

  // RIGHT BUTTON
  if (rightNow != lastRight && (now1 - lastRightTime) > DEBOUNCE_DELAY) {
    if (rightNow == LOW)
      bleMouse.press(MOUSE_RIGHT);
    else
      bleMouse.release(MOUSE_RIGHT);
    lastRightTime = now1;
  }

  lastLeft  = leftNow;
  lastRight = rightNow;

  delay(10);
}
