#include <Arduino.h>

#include "config.h"
#include "bumpers.h"

namespace {
bool leftBumperHit = false;
bool rightBumperHit = false;
}

void setupBumpers() {
  pinMode(LEFT_BUMPER_PIN, INPUT);
  pinMode(RIGHT_BUMPER_PIN, INPUT);
}

void updateBumpers() {
  leftBumperHit = (digitalRead(LEFT_BUMPER_PIN) == HIGH);
  rightBumperHit = (digitalRead(RIGHT_BUMPER_PIN) == HIGH);
}

bool isLeftBumperHit() {
  return leftBumperHit;
}

bool isRightBumperHit() {
  return rightBumperHit;
}

bool isSideBumperHit() {
  return leftBumperHit || rightBumperHit;
}
