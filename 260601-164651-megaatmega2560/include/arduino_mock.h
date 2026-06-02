#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <stdint.h>

typedef unsigned long unsigned_long_t;
typedef unsigned long millis_t;

#define INPUT 0
#define INPUT_PULLUP 1
#define CHANGE 2

// Analog pin aliases used in config.h
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

#ifndef PI
#define PI 3.14159265358979323846
#endif

// Minimal prototypes used by the code under test. Tests provide implementations.
extern int analogRead(int pin);
extern unsigned long millis();
extern void pinMode(int pin, int mode);
extern int digitalRead(int pin);

inline int constrainInt(int x, int a, int b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Provide a macro-like name used in code
#define constrain(x, a, b) constrainInt((x), (a), (b))

#endif
