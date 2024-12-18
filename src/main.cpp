#include <Arduino.h>

// Definition of pinout
#define RIGHT_MTR_FRWD 36
#define RIGHT_MTR_BCKWD 37
#define PWM_RIGHT_MTR 1

#define LEFT_MTR_FRWD 45
#define LEFT_MTR_BCKWD 35
#define LEFT_RIGHT_MTR 47

#define MOTOR_STANDBY 46

#define LIDAR_PWM 21

void setup()
{
  Serial1.begin(115200);
  Serial.begin(230400);

  pinMode(RIGHT_MTR_FRWD, OUTPUT);
  pinMode(RIGHT_MTR_BCKWD, OUTPUT);
  pinMode(PWM_RIGHT_MTR, OUTPUT);

  pinMode(LEFT_MTR_FRWD, OUTPUT);
  pinMode(LEFT_MTR_BCKWD, OUTPUT);
  pinMode(LEFT_RIGHT_MTR, OUTPUT);

  pinMode(MOTOR_STANDBY, OUTPUT);
  digitalWrite(MOTOR_STANDBY, 1);

  pinMode(LIDAR_PWM, OUTPUT);
  analogWrite(LIDAR_PWM, 255);
}

void loop()
{
  digitalWrite(RIGHT_MTR_FRWD, 0);
  digitalWrite(RIGHT_MTR_BCKWD, 0);
  digitalWrite(LEFT_MTR_FRWD, 0);
  digitalWrite(LEFT_MTR_BCKWD, 0);
  analogWrite(PWM_RIGHT_MTR, 100);
  analogWrite(LEFT_RIGHT_MTR, 100);
  delay(1000);
  digitalWrite(RIGHT_MTR_FRWD, 0);
  digitalWrite(RIGHT_MTR_BCKWD, 0);
  digitalWrite(LEFT_MTR_FRWD, 0);
  digitalWrite(LEFT_MTR_BCKWD, 0);
  delay(1000);

  // int value = Serial.read();
  // Serial1.println(value);
}
