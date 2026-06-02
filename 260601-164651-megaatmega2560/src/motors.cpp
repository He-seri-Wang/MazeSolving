#include <Arduino.h>

#include "config.h"
#include "motors.h"

namespace {
void setMotorA(int direction, int speed) {
  speed = constrain(speed, 0, 255);

  if (direction == 0) {
    digitalWrite(BRK_A, HIGH);
    analogWrite(PWM_A, 0);
    return;
  }

  digitalWrite(BRK_A, LOW);
  digitalWrite(DIR_A, direction > 0 ? HIGH : LOW);
  analogWrite(PWM_A, speed);
}

void setMotorB(int direction, int speed) {
  speed = constrain(speed, 0, 255);

  if (direction == 0) {
    digitalWrite(BRK_B, HIGH);
    analogWrite(PWM_B, 0);
    return;
  }

  digitalWrite(BRK_B, LOW);
  digitalWrite(DIR_B, direction > 0 ? HIGH : LOW);
  analogWrite(PWM_B, speed);
}
}

void setupMotors() {
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(BRK_A, OUTPUT);

  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRK_B, OUTPUT);
}

void stopForwardBackMotor() {
  setMotorA(0, 0);
}

void stopLeftRightMotor() {
  setMotorB(0, 0);
}

void stopAllMotors() {
  stopForwardBackMotor();
  stopLeftRightMotor();
}

void moveForward() {
  setMotorA(1, MOTOR_SPEED);
}

void moveBackward() {
  setMotorA(-1, MOTOR_SPEED);
}

void moveRight() {
  setMotorB(1, MOTOR_SPEED);
}

void moveLeft() {
  setMotorB(-1, MOTOR_SPEED);
}

void moveSideByDirection(int direction) {
  if (direction >= 0) {
    moveRight();
  } else {
    moveLeft();
  }
}

void applyBrake(unsigned long duration) {
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  digitalWrite(BRK_A, HIGH);
  digitalWrite(BRK_B, HIGH);
  delay(duration);
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, LOW);
}
