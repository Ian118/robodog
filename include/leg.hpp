#pragma once

#include <ArduinoEigen.h>

#include "constants.hpp"
#include "servo.hpp"

class Leg {
 public:
  Leg(Servo shoulder, Servo upper_leg, Servo lower_leg, Eigen::Vector3f offset);

  void disable();

  void operator=(const Eigen::Vector3f &p);

  const Eigen::Vector3f offset;

 private:
  Servo shoulder, upperLeg, lowerLeg;
  float shoulderAngle, upperAngle, lowerAngle;
  float shoulderOffset{shoulderOffset};
};
