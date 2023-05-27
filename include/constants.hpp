#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <ArduinoEigen.h>

constexpr float upperLen{0.05f};             // meters
constexpr float lowerLen{0.05f};             // meters
constexpr float footRadius{0.01f};           // meters
constexpr float shoulderOffset{0.0119685f};  // meters

const Eigen::Vector3f legOffset{0.036672f, 0.0434685f, 0.0f};  // meters