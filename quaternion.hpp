#ifndef GYRO_HPP
#define GYRO_HPP

#include <complex>

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
        return {w * q.w - i * q.i - j * q.j - k * q.k,
                w * q.i + i * q.w + j * q.k - k * q.j,
                w * q.j - i * q.k + j * q.w + k * q.i,
                w * q.k + i * q.j - j * q.i + k * q.w};
    }

    // Inverse quaternion
    quaternion operator!()
    {
        return {w, -i, -j, -k};
    }

    // Negative quaternion
    quaternion operator-()
    {
        return {-w, -i, -j, -k};
    }

    // Quaternion equality
    bool operator==(const quaternion &q)
    {
        return (w == q.w && i == q.i && j == q.j && k == q.k) || (w == -q.w && i == -q.i && j == -q.j && k == -q.k);
    }

    quaternion operator+(quaternion q)
    {
        return {w + q.w, i + q.i, j + q.j, k + q.k};
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

    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

// Quaternion to Euler Angles
quaternion::operator euler() const
{
    euler angles = {0.0f, 0.0f, 0.0f};

    // roll (x-axis rotation)
    // atan2(sin(r)cos(p),cos(r)cos(p))
    angles.roll = atan2f(2.0f * (w * i + j * k), 1.0f - 2.0f * (i * i + j * j));

    // pitch (y-axis rotation)
    // asin(sin(p))
    angles.pitch = asinf(MAX(MIN(2.0f * (w * j - k * i), 1.0f), -1.0f));

    // yaw (z-axis rotation)
    // atan2(sin(y)cos(p),cos(y)cos(p))
    angles.yaw = atan2f(2.0f * (w * k + i * j), 1.0f - 2.0f * (j * j + k * k));

    return angles;
}

inline quaternion operator*(const quaternion &q, float x)
{
    return {x * q.w, x * q.i, x * q.j, x * q.k};
}

inline quaternion operator*(float x, const quaternion &q)
{
    return {x * q.w, x * q.i, x * q.j, x * q.k};
}

void printQuaternion(quaternion q)
{
    Serial.printf("%f, %f, %f, %f ", q.w, q.i, q.j, q.k);
}

void printlnQuaternion(quaternion q)
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

// quaternion slerp(quaternion q0, quaternion q1, float t)
// {
//     float cosphi = q0.w * q1.w + q0.i * q1.i + q0.j * q1.j + q0.k * q1.k; // angle/2
//     if (cosphi < 1.0f)
//     {
//         float phi = acosf(cosphi);
//         float inv_sinphi = 1.0f / sin(phi);
//         float c0 = sin((1 - t) * phi) * inv_sinphi;
//         float c1 = sin(t * phi) * inv_sinphi;
//         return c0 * q0 + c1 * q1;
//     }
//     else
//         return q0;
// }

quaternion pow(const quaternion &q, float x)
{
    float phi = acosf(q.w);
    float sinphi = sin(phi);
    if (sinphi == 0.0f)
        return q;
    float c = sin(x * phi) / sinphi;
    return {cosf(x * phi), c * q.i, c * q.j, c * q.k};
}

quaternion slerp(quaternion q0, quaternion q1, float t)
{
    return q0 * pow(!q0 * q1, t);
}

#endif