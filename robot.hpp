#include "leg.hpp"

class Robot
{
    const unsigned cycle_time = 2000U;

public:
    quaternion orientation = {1.0f}, correction = {1.0f};
    leg m_right_front, m_left_front, m_right_back, m_left_back;
    leg *legs[4] = {&m_right_front, &m_left_back, &m_left_front, &m_right_back};
    float v_x = 0.0f, v_y = 0.0f, v_yaw = 0.0f; // velocities in m/s or rad/s

    bool relax;

    Robot(leg right_front, leg left_front, leg right_back, leg left_back, quaternion leg_offset, float upper_leg_len, float lower_leg_len, float foot_radius, float shoulder_offset)
        : m_right_front{right_front}, m_left_front{left_front}, m_right_back{right_back}, m_left_back{left_back}
    {
        m_right_front.offset = {0, leg_offset.i, leg_offset.j, leg_offset.k};
        m_left_front.offset = {0, leg_offset.i, -leg_offset.j, leg_offset.k};
        m_right_back.offset = {0, -leg_offset.i, leg_offset.j, leg_offset.k};
        m_left_back.offset = {0, -leg_offset.i, -leg_offset.j, leg_offset.k};
        unsigned i = 0;
        for (leg *cur_leg : legs)
        {
            cur_leg->offset = {0, (i % 2 ? -1.0f : 1.0f) * leg_offset.i, (i == 1 || i == 2 ? -1.0f : 1.0f) * leg_offset.j, leg_offset.k};
            cur_leg->upper_leg_len = upper_leg_len;
            cur_leg->lower_leg_len = lower_leg_len;
            cur_leg->foot_radius = foot_radius;
            cur_leg->shoulder_offset = (i == 1 || i == 2 ? -1.0f : 1.0f) * shoulder_offset;

            i++;
        }
    };

    void process()
    {
        unsigned long current_time;
        float t;
        unsigned i = 0;
        quaternion pos;

        const unsigned long start_time = millis();
        const float cycle_hz = 1000.0f / cycle_time;

        while (1)
        {
            i = 0;
            for (leg *cur_leg : legs)
            {
                pos = {};
                if (relax)
                    *cur_leg = false;
                else
                {
                    if (v_x || v_y || v_yaw) // check if there is a velocity
                    {
                        t = (float)((current_time + i * cycle_time / 4) % cycle_time) * 4.0f / cycle_time; // get current time from 0 to 1
                        float path_x = v_x * cycle_hz, path_y = v_y * cycle_hz;
                        if (t <= 1.0f)
                        {
                            t *= M_PI;
                            pos.i = cur_leg->offset.i + path_x * cosf(t);
                            pos.j = cur_leg->offset.j + path_y * cosf(t);
                            pos.k = cur_leg->offset.k - sqrt(v_x * v_x + v_y * v_y) * cycle_hz * sinf(t);
                            // Serial.printf("%f\n", pos.j);
                        }
                        else
                        {
                            t = (t - 1.0f) / 3.0f;
                            pos.i = cur_leg->offset.i + path_x * (2.0f * t - 1.0f);
                            pos.j = cur_leg->offset.j + path_y * (2.0f * t - 1.0f);
                            pos.k = cur_leg->offset.k;
                            if (i == 0)
                                // Serial.printf("%f, %f\n", t, (1.0f - 2.0f * t));
                                printVector(pos);
                        }
                        // pos.i = cur_leg->offset.i + path_x * path_radius * cosf(2 * M_PI * t);
                        // pos.j = cur_leg->offset.j + path_y * path_radius * cosf(2 * M_PI * t);
                        // pos.k = cur_leg->offset.k - 0.035f * 0.5f * (sinf(2 * M_PI * t) - 0.25f * cosf(4 * M_PI * t) + 0.75f);
                        *cur_leg = pos;
                    }
                    else if (orientation.w != 1.0f)
                    {
                        pos = !orientation * cur_leg->offset * orientation;
                        *cur_leg = pos;
                    }
                    else
                    {
                        pos = cur_leg->offset;
                        *cur_leg = correction * pos * !correction;
                    }
                }
                i++;
            }
            current_time = millis() - start_time;
            delay(2);
        }
    }
};