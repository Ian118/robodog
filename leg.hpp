#ifndef LEG_HPP
#define LEG_HPP

#define MIN(X, Y) (X < Y ? X : Y)
#define MAX(X, Y) (X > Y ? X : Y)

#include <Adafruit_PWMServoDriver.h>
// #include "gyro.hpp"

struct servo_t
{
    unsigned char port;
    unsigned min_pulse;
    unsigned max_pulse;
    float min_val;
    float max_val;
};

// position (in radians) to servo pulse
unsigned position_to_pulse(float pos, struct servo_t servo);

class leg
{
    // Adafruit_PWMServoDriver m_pwm;
    const struct servo_t m_shoulder, m_upper_leg, m_lower_leg;
    const float m_upper_leg_len, m_lower_leg_len, m_foot_radius, m_shoulder_offest; // in meters
    float m_x, m_y, m_z;
    Adafruit_PWMServoDriver *m_pwm;
    // temporary variables
    float m_d2, m_servo1, m_servo2, m_servo3;

public:
    leg(struct servo_t shoulder, struct servo_t upper_leg, struct servo_t lower_leg, Adafruit_PWMServoDriver *pwm, float upper_leg_len, float lower_leg_len, float foot_radius, float shoulder_offset);

    void move_to(float x, float y, float z);
    // void move_to(quaternion q);
};
#endif