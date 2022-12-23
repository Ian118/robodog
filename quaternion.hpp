#ifndef GYRO_HPP
#define GYRO_HPP

typedef struct euler euler;
typedef struct quaternion quaternion;

struct euler
{
    float yaw, pitch, roll;

    operator quaternion() const;
};

struct quaternion
{
    float w, i, j, k;

    // Hamilton product of two quaternions
    quaternion operator*(const quaternion &q)
    {
        return (quaternion){w * q.w - i * q.i - j * q.j - k * q.k,
                            w * q.i + i * q.w + j * q.k - k * q.j,
                            w * q.j - i * q.k + j * q.w + k * q.i,
                            w * q.k + i * q.j - j * q.i + k * q.w};
    }

    // Inverse quaternion
    quaternion operator!()
    {
        return (quaternion){w, -i, -j, -k};
    }

    operator euler() const;
};

// Euler Angles to Quaternion
euler::operator quaternion() const
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    return (quaternion){cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

// Quaternion to Euler Angles
quaternion::operator euler() const
{
    euler angles = {0, 0, 0};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * i + j * k);
    double cosr_cosp = 1 - 2 * (i * i + j * j);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * j - k * i);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI_2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * k + i * j);
    double cosy_cosp = 1 - 2 * (j * j + k * k);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void printQuaternion(quaternion q)
{
    Serial.printf("%f, %f, %f, %f\n", q.w, q.i, q.j, q.k);
}

void printVector(quaternion q)
{
    Serial.printf("%f, %f, %f\n", q.i, q.j, q.k);
}

void printEuler(euler e)
{
    Serial.printf("%f, %f, %f\n", e.roll, e.pitch, e.yaw);
}
#endif