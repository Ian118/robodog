#include "../include/servo.hpp"

Adafruit_PWMServoDriver* pwm = new Adafruit_PWMServoDriver();

Servo::Servo(unsigned char port, unsigned minPulse, unsigned maxPulse,
             float minVal, float maxVal)
    : port{port},
      minPulse{minPulse},
      maxPulse{maxPulse},
      minVal{minVal},
      maxVal{maxVal} {}

void Servo::operator=(const float pos) {
  float pulse =
      (maxPulse - minPulse) / (maxVal - minVal) * (pos - minVal) + minPulse;
  pwm->setPWM(port, 0, constrain(pulse, minPulse, maxPulse));
}

void Servo::disable() { pwm->setPWM(port, 0, 0); }