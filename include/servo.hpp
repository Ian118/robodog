#pragma once

#include <algorithm>

#include "constants.hpp"

extern Adafruit_PWMServoDriver* pwm;

class Servo {
 public:
  Servo(unsigned char port, unsigned minPulse, unsigned maxPulse, float minVal,
        float maxVal);

  void disable();

  void operator=(const float pos);

 private:
  unsigned char port;
  unsigned minPulse;
  unsigned maxPulse;
  float minVal;
  float maxVal;
};
