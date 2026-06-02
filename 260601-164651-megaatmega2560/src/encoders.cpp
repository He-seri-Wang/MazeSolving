#include <Arduino.h>
#include <EnableInterrupt.h>

#include "config.h"
#include "encoders.h"

namespace {
volatile long encoderA = 0;
volatile long encoderB = 0;
volatile long encoderC = 0;
volatile long encoderD = 0;

void encoderA_ISR()  { (digitalRead(ENCA1) == digitalRead(ENCA2)) ? encoderA++ : encoderA--; }
void encoderA2_ISR() { (digitalRead(ENCA1) != digitalRead(ENCA2)) ? encoderA++ : encoderA--; }
void encoderB_ISR()  { (digitalRead(ENCB1) == digitalRead(ENCB2)) ? encoderB++ : encoderB--; }
void encoderB2_ISR() { (digitalRead(ENCB1) != digitalRead(ENCB2)) ? encoderB++ : encoderB--; }
void encoderC_ISR()  { (digitalRead(ENCC1) == digitalRead(ENCC2)) ? encoderC++ : encoderC--; }
void encoderC2_ISR() { (digitalRead(ENCC1) != digitalRead(ENCC2)) ? encoderC++ : encoderC--; }
void encoderD_ISR()  { (digitalRead(ENCD1) == digitalRead(ENCD2)) ? encoderD++ : encoderD--; }
void encoderD2_ISR() { (digitalRead(ENCD1) != digitalRead(ENCD2)) ? encoderD++ : encoderD--; }
}

void setupEncoders() {
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(ENCC1, INPUT_PULLUP);
  pinMode(ENCC2, INPUT_PULLUP);
  pinMode(ENCD1, INPUT_PULLUP);
  pinMode(ENCD2, INPUT_PULLUP);

  enableInterrupt(ENCA1, encoderA_ISR,  CHANGE);
  enableInterrupt(ENCA2, encoderA2_ISR, CHANGE);
  enableInterrupt(ENCB1, encoderB_ISR,  CHANGE);
  enableInterrupt(ENCB2, encoderB2_ISR, CHANGE);
  enableInterrupt(ENCC1, encoderC_ISR,  CHANGE);
  enableInterrupt(ENCC2, encoderC2_ISR, CHANGE);
  enableInterrupt(ENCD1, encoderD_ISR,  CHANGE);
  enableInterrupt(ENCD2, encoderD2_ISR, CHANGE);
}

void resetEncoders() {
  noInterrupts();
  encoderA = 0;
  encoderB = 0;
  encoderC = 0;
  encoderD = 0;
  interrupts();
}

void resetForwardEncoders() {
  noInterrupts();
  encoderA = 0;
  encoderB = 0;
  interrupts();
}

void resetSideEncoders() {
  noInterrupts();
  encoderC = 0;
  encoderD = 0;
  interrupts();
}

float getForwardTravelCm() {
  noInterrupts();
  long a = encoderA;
  long b = encoderB;
  interrupts();

  float avgCounts = (abs(a) + abs(b)) * 0.5f;
  return (avgCounts * MM_PER_COUNT) / 10.0f;
}

float getSideTravelCm() {
  noInterrupts();
  long c = encoderC;
  long d = encoderD;
  interrupts();

  float avgCounts = (abs(c) + abs(d)) * 0.5f;
  return (avgCounts * MM_PER_COUNT) / 10.0f;
}
