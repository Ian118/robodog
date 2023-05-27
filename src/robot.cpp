#include "../include/robot.hpp"

#include <iostream>

void Robot::periodic() {
  unsigned long current_time;
  float t;
  unsigned i = 0;
  Eigen::Vector3f pos;

  const unsigned long start_time = millis();
  const float cycle_hz = 1000.0f / cycle_time;

  while (1) {
    i = 0;
    for (Leg leg : legs) {
      pos = {};
      if (relax)
        leg.disable();
      else {
        if (v_x || v_y || v_yaw)  // check if there is a velocity
        {
          t = (float)((current_time + i * cycle_time / 4) % cycle_time) * 4.0f /
              cycle_time;  // get current time from 0 to 1
          float path_x = v_x * cycle_hz, path_y = v_y * cycle_hz;
          if (t <= 1.0f) {
            t *= M_PI;
            pos.x() = leg.offset.x() + path_x * cosf(t);
            pos.y() = leg.offset.y() + path_y * cosf(t);
            pos.z() = leg.offset.z() -
                      sqrt(v_x * v_x + v_y * v_y) * cycle_hz * sinf(t);
            // Serial.printf("%f\n", pos.y());
          } else {
            t = (t - 1.0f) / 3.0f;
            pos.x() = leg.offset.x() + path_x * (2.0f * t - 1.0f);
            pos.y() = leg.offset.y() + path_y * (2.0f * t - 1.0f);
            pos.z() = leg.offset.z();
            if (i == 0)
              // Serial.printf("%f, %f\n", t, (1.0f - 2.0f * t));
              std::cout << pos << std::endl;
          }
          // pos.x() = cur_leg.offset.x() + path_x * path_radius * cosf(2 * M_PI
          // * t); pos.y() = cur_leg.offset.y() + path_y * path_radius * cosf(2
          // * M_PI * t); pos.z() = cur_leg.offset.z() - 0.035f * 0.5f * (sinf(2
          // * M_PI * t) - 0.25f * cosf(4 * M_PI * t) + 0.75f);
          leg = pos;
        } else if (orientation.w() != 1.0f) {
          pos = orientation * leg.offset;
          leg = pos;
        } else {
          pos = leg.offset;
          leg = correction.inverse() * pos;
        }
      }
      i++;
    }
    current_time = millis() - start_time;
    delay(2);
  }
}