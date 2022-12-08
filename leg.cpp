#include "leg.h"

unsigned position_to_pulse(float pos, struct servo_t servo)
{
    return (unsigned)MAX(servo.min_pulse, MIN(servo.max_pulse, (servo.max_pulse - servo.min_pulse) / (servo.max_val - servo.min_val) * (pos - servo.min_val) + servo.min_pulse));
}

leg::leg(struct servo_t shoulder, struct servo_t upper_leg, struct servo_t lower_leg, Adafruit_PWMServoDriver *pwm, float upper_leg_len, float lower_leg_len, float foot_radius)
    : m_shoulder{shoulder}, m_upper_leg{upper_leg}, m_lower_leg{lower_leg}, m_pwm{pwm}, m_upper_leg_len{upper_leg_len}, m_lower_leg_len{lower_leg_len}, m_foot_radius{foot_radius}
{
}

void leg::move_to(float x, float y, float z)
{
    m_x = x, m_y = y, m_z = z;
    z -= m_foot_radius;
    m_d2 = x * x + z * z; // Distance b/w circles squared (for efficiency)
    m_c1 = m_upper_leg_len * m_upper_leg_len - m_lower_leg_len * m_lower_leg_len + m_d2;
    m_c2 = m_c1 / (2.0 * m_d2);
    if (4 * m_upper_leg_len * m_upper_leg_len >= m_c1 * m_c1 / m_d2) // Is there a solution? To optimize later
    {
        m_c3 = sqrtf(4 * m_upper_leg_len * m_upper_leg_len - m_c1 * m_c1 / m_d2) / (2 * sqrtf(m_d2));
        m_servo1 = -atanf((m_c2 * z + m_c3 * x) / (m_c2 * x - m_c3 * z));
        m_servo2 = M_PI - acos((m_upper_leg_len * m_upper_leg_len + m_lower_leg_len * m_lower_leg_len - m_d2) / (2.0 * m_upper_leg_len * m_lower_leg_len));
        m_pwm->setPWM(m_upper_leg.port, 0, position_to_pulse(-m_servo1, m_upper_leg));
        m_pwm->setPWM(m_lower_leg.port, 0, position_to_pulse(m_servo2, m_lower_leg));
        m_pwm->setPWM(m_shoulder.port, 0, position_to_pulse(0, m_shoulder));
    }

    // else
    // {
    //     m_pwm->setPWM(m_upper_leg.port, 0, position_to_pulse(0, m_upper_leg));
    //     m_pwm->setPWM(m_lower_leg.port, 0, position_to_pulse(0, m_lower_leg));
    //     m_pwm->setPWM(m_shoulder.port, 0, position_to_pulse(0, m_shoulder));
    // }
}