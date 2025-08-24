#pragma once
#include "Arduino.h"
// #define ENC_TICKS_PER_REV 4455 // 660

class Encoder
{
private:
  unsigned int pinA;
  unsigned int pinB;
  volatile long ticks = 0;
  int increment = 0;
  short encTicksPerRev = 0;
public:

  Encoder(
      unsigned int pinA,
      unsigned int pinB,
      short encTicksPerRev,
      bool isClockwise = false);
  void triggerA(); //trigger to add or subtract ticks
  void triggerB(); //trigger to add or subtract ticks
  volatile long getTicks(); //return ticks
  void reset(); //reset ticks to 0
  int getPinA(); // return pin A name
  int getPinB(); // return pin B name
  double getAngle(); //return angle in degrees
};
