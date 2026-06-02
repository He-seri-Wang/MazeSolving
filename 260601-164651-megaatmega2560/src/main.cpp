#include <Arduino.h>
#include "robot_state.h"

void setup() {
  setupRobot();
}

void loop() {
  updateRobot();
  printDebug();
}
