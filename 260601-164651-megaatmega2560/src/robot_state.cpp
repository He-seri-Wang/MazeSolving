#include <Arduino.h>

#include "bumpers.h"
#include "config.h"
#include "encoders.h"
#include "ir_sensor.h"
#include "motors.h"
#include "robot_state.h"

namespace {
enum RobotState {
  STATE_SIDE_MOVE,
  STATE_VERIFY_GAP,
  STATE_FORWARD_MOVE
};

RobotState robotState = STATE_SIDE_MOVE;
int sideDirection = 1;

int gapSampleTotal = 0;
int gapSampleGood = 0;
float gapSampleSum = 0.0f;

unsigned long lastPrintTime = 0;

void resetGapSamples() {
  gapSampleTotal = 0;
  gapSampleGood = 0;
  gapSampleSum = 0.0f;
}

void updateSideDirectionFromBumpers() {
  if (isLeftBumperHit()) {
    sideDirection = 1;
  } else if (isRightBumperHit()) {
    sideDirection = -1;
  }
}
}

void setupRobot() {
  Serial.begin(115200);

  setupMotors();
  setupEncoders();
  setupIrSensor();
  setupBumpers();

  stopAllMotors();
  resetEncoders();
  resetGapSamples();

  robotState = STATE_SIDE_MOVE;
  sideDirection = 1;

  Serial.println("System ready.");
}

void updateRobot() {
  bool sensorUpdated = updateIrSensor();
  updateBumpers();

  switch (robotState) {
    case STATE_SIDE_MOVE:
      stopForwardBackMotor();
      updateSideDirectionFromBumpers();
      moveSideByDirection(sideDirection);

      if (getGapDistanceCm() >= GAP_DISTANCE_THRESHOLD_CM) {
        resetSideEncoders();
        resetGapSamples();
        robotState = STATE_VERIFY_GAP;
      }
      break;

    case STATE_VERIFY_GAP:
      stopForwardBackMotor();

      if (isSideBumperHit()) {
        updateSideDirectionFromBumpers();
        resetSideEncoders();
        resetGapSamples();
        robotState = STATE_SIDE_MOVE;
        break;
      }

      moveSideByDirection(sideDirection);

      if (sensorUpdated) {
        float gapDistance = getGapDistanceCm();
        gapSampleTotal++;
        gapSampleSum += gapDistance;

        if (gapDistance >= GAP_DISTANCE_THRESHOLD_CM) {
          gapSampleGood++;
        }
      }

      if (getSideTravelCm() >= GAP_VERIFY_TRAVEL_CM) {
        float ratio = gapSampleTotal > 0 ?
          static_cast<float>(gapSampleGood) / gapSampleTotal :
          0.0f;
        float average = gapSampleTotal > 0 ?
          gapSampleSum / gapSampleTotal :
          0.0f;
        bool gapConfirmed = gapSampleTotal >= GAP_MIN_SAMPLE_COUNT &&
          ratio >= GAP_MIN_SAMPLE_RATIO &&
          average >= GAP_DISTANCE_THRESHOLD_CM;

        if (gapConfirmed) {
          stopLeftRightMotor();
          resetForwardEncoders();
          robotState = STATE_FORWARD_MOVE;
        } else {
          robotState = STATE_SIDE_MOVE;
        }
      }
      break;

    case STATE_FORWARD_MOVE:
      stopLeftRightMotor();

      if (getStopDistanceCm() <= FRONT_STOP_DISTANCE_CM) {
        stopForwardBackMotor();
        robotState = STATE_SIDE_MOVE;
        break;
      }

      moveForward();
      break;
  }
}

void printDebug() {
  unsigned long now = millis();
  if (now - lastPrintTime < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintTime = now;

  Serial.print("State=");
  switch (robotState) {
    case STATE_SIDE_MOVE: Serial.print("SIDE"); break;
    case STATE_VERIFY_GAP: Serial.print("VERIFY"); break;
    case STATE_FORWARD_MOVE: Serial.print("FORWARD"); break;
  }

  Serial.print(" | GapD=");
  Serial.print(getGapDistanceCm(), 2);
  Serial.print(" | StopD=");
  Serial.print(getStopDistanceCm(), 2);
  Serial.print(" | Side=");
  Serial.print(getSideTravelCm(), 2);
  Serial.print(" | GapAvg=");

  float ratio = gapSampleTotal > 0 ?
    static_cast<float>(gapSampleGood) / gapSampleTotal :
    0.0f;
  float average = gapSampleTotal > 0 ?
    gapSampleSum / gapSampleTotal :
    0.0f;

  Serial.print(average, 2);
  Serial.print(" | GapRatio=");
  Serial.print(ratio, 2);
  Serial.print(" | Samples=");
  Serial.print(gapSampleGood);
  Serial.print("/");
  Serial.print(gapSampleTotal);
  Serial.print(" | Dir=");
  Serial.print(sideDirection > 0 ? "RIGHT" : "LEFT");
  Serial.println();
}
