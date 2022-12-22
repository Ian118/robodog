#ifndef LEG_HPP
#define LEG_HPP

#define MIN(X, Y) (X < Y ? X : Y)
#define MAX(X, Y) (X > Y ? X : Y)

#include <Adafruit_PWMServoDriver.h>
#include "quaternion.hpp"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct servo_t
{
    unsigned char port;
    unsigned min_pulse;
    unsigned max_pulse;
    float min_val;
    float max_val;
    void operator=(const float pos)
    {
        Serial.printf("Servo %d to %d\n", port, (unsigned)MAX(min_pulse, MIN(max_pulse, (max_pulse - min_pulse) / (max_val - min_val) * (pos - min_val) + min_pulse)));
        pwm.setPWM(port, 0, (unsigned)MAX(min_pulse, MIN(max_pulse, (max_pulse - min_pulse) / (max_val - min_val) * (pos - min_val) + min_pulse)));
    }
};

struct leg
{
    struct servo_t shoulder, upper_leg, lower_leg;
    quaternion offset;
    float upper_leg_len, lower_leg_len, foot_radius, shoulder_offset; // in meters

    void operator=(const quaternion &q)
    {
        float x = q.i - offset.i, y = q.j - offset.j + shoulder_offset, z = q.k - foot_radius;

        float z1 = z;
        float shoulder_theta = atan2f(z, y) - acosf(shoulder_offset / sqrtf(z * z + y * y));
        z = sqrtf(z * z + y * y - shoulder_offset * shoulder_offset);
        float d2 = x * x + z * z;
        float upper_theta = atan2f(z, x) - acosf((upper_leg_len * upper_leg_len + d2 - lower_leg_len * lower_leg_len) / (2.0 * upper_leg_len * sqrtf(d2)));
        float lower_theta = M_PI - acosf((upper_leg_len * upper_leg_len + lower_leg_len * lower_leg_len - d2) / (2.0 * upper_leg_len * lower_leg_len));

        shoulder = shoulder_theta;
        upper_leg = upper_theta;
        lower_leg = lower_theta;
        Serial.printf("%f, %f, %f, %f\n", atan2f(z1, y), -acosf(shoulder_offset / sqrtf(z1 * z1 + y * y)), z1, y);
        Serial.printf("%f, %f, %f; %f, %f, %f\n", x, y, z, shoulder_theta, upper_theta, lower_theta);
    }
};

#endif