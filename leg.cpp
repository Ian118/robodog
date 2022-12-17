#include "leg.hpp"

unsigned position_to_pulse(float pos, struct servo_t servo)
{
    return (unsigned)MAX(servo.min_pulse, MIN(servo.max_pulse, (servo.max_pulse - servo.min_pulse) / (servo.max_val - servo.min_val) * (pos - servo.min_val) + servo.min_pulse));
}

leg::leg(struct servo_t shoulder, struct servo_t upper_leg, struct servo_t lower_leg, Adafruit_PWMServoDriver *pwm, float upper_leg_len, float lower_leg_len, float foot_radius, float shoulder_offset)
    : m_shoulder{shoulder}, m_upper_leg{upper_leg}, m_lower_leg{lower_leg}, m_pwm{pwm}, m_upper_leg_len{upper_leg_len}, m_lower_leg_len{lower_leg_len}, m_foot_radius{foot_radius}, m_shoulder_offest{shoulder_offset}
{
}

void leg::move_to(float x, float y, float z)
{
    m_x = x, m_y = y, m_z = z;
    z -= m_foot_radius;
    // Calculate y first, then x and z
    m_servo1 = atan2f(z, y) - acosf(m_shoulder_offest / sqrtf(z * z + y * y));
    // Change z since y changed the height
    z = sqrtf(z * z + y * y - m_shoulder_offest * m_shoulder_offest);
    m_d2 = x * x + z * z;
    m_servo2 = atan2f(z, x) - acosf((m_upper_leg_len * m_upper_leg_len + m_d2 - m_lower_leg_len * m_lower_leg_len) / (2.0 * m_upper_leg_len * sqrtf(m_d2)));
    m_servo3 = M_PI - acosf((m_upper_leg_len * m_upper_leg_len + m_lower_leg_len * m_lower_leg_len - m_d2) / (2.0 * m_upper_leg_len * m_lower_leg_len));
    m_pwm->setPWM(m_shoulder.port, 0, position_to_pulse(m_servo1, m_shoulder));
    m_pwm->setPWM(m_upper_leg.port, 0, position_to_pulse(-m_servo2, m_upper_leg));
    m_pwm->setPWM(m_lower_leg.port, 0, position_to_pulse(m_servo3, m_lower_leg));
}

// void leg::move_to(quaternion q)
// {
//     move_to(q.i, q.j, q.k);
// }