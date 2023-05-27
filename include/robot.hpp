#pragma once

#include <ArduinoEigen.h>

#include "leg.hpp"

class Robot {
 public:
  Eigen::Quaternionf orientation{}, correction{};
  float v_x{0.0f}, v_y{0.0f}, v_yaw{0.0f};  // velocities in m/s or rad/s

  void periodic();

 private:
  Leg rightFront{Servo{9, 248 - 180, 248 + 180, M_PI_2, -M_PI_2},
                 Servo{10, 456 - 360, 456, -M_PI_2, M_PI_2},
                 Servo{11, 458 - 360, 458, M_PI, 0}, legOffset},
      leftFront{Servo{6, 272 - 180, 272 + 180, M_PI_2, -M_PI_2},
                Servo{7, 64, 64 + 360, M_PI_2, -M_PI_2},
                Servo{8, 96, 96 + 360, 0, M_PI},
                {legOffset.x(), -legOffset.y(), legOffset.z()}},
      rightBack{Servo{3, 220 - 180, 220 + 180, -M_PI_2, M_PI_2},
                Servo{4, 400 - 360, 400, -M_PI_2, M_PI_2},
                Servo{5, 450 - 360, 450, M_PI, 0},
                {-legOffset.x(), legOffset.y(), legOffset.z()}},
      leftBack{Servo{0, 294 - 180, 294 + 180, -M_PI_2, M_PI_2},
               Servo{1, 80, 80 + 360, M_PI_2, -M_PI_2},
               Servo{2, 92, 92 + 360, 0, M_PI},
               {-legOffset.x(), -legOffset.y(), legOffset.z()}};
  const Leg legs[4]{rightFront, leftFront, rightBack, leftBack};
  const unsigned cycle_time{2000U};

  bool relax{false};
};
