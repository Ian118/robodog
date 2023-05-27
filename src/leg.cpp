#include "../include/leg.hpp"

Leg::Leg(Servo shoulder, Servo upperLeg, Servo lowerLeg, Eigen::Vector3f offset)
    : shoulder{shoulder},
      upperLeg{upperLeg},
      lowerLeg{lowerLeg},
      offset{offset},
      shoulderOffset{copysignf(shoulderOffset, offset.y())} {}

void Leg::operator=(const Eigen::Vector3f &p) {
  float x = p.x() - offset.x(), y = p.y() - offset.y() + shoulderOffset,
        z = sqrtf(upperLen * upperLen + lowerLen * lowerLen) + p.z() -
            footRadius;

  shoulderAngle = atan2f(z, y) - acosf(shoulderOffset / sqrtf(z * z + y * y));
  z = sqrtf(z * z + y * y - shoulderOffset * shoulderOffset);
  float d2 = x * x + z * z;
  upperAngle = M_PI - atan2f(z, x) -
               acosf((upperLen * upperLen + d2 - lowerLen * lowerLen) /
                     (2.0f * upperLen * sqrtf(d2)));
  lowerAngle = M_PI - acosf((upperLen * upperLen + lowerLen * lowerLen - d2) /
                            (2.0f * upperLen * lowerLen));

  shoulder = shoulderAngle;
  upperLeg = upperAngle;
  lowerLeg = lowerAngle;
}

void Leg::disable() {
  shoulder.disable();
  upperLeg.disable();
  lowerLeg.disable();
}