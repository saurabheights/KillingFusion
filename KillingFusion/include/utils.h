#ifndef UTILS_H
#define UTILS_H

// ToDo: Use template.

inline float interpolate1D(float v_0, float v_1, float x)
{
    return v_0 * (1 - x) + v_1 * x;
}

inline float interpolate2D(float v_00, float v_01, float v_10, float v_11, float x, float y)
{
    float s = interpolate1D(v_00, v_01, x);
    float t = interpolate1D(v_10, v_11, x);
    return interpolate1D(s, t, y);
}

inline float interpolate3D(float v_000, float v_001, float v_010, float v_011,
                    float v_100, float v_101, float v_110, float v_111,
                    float x, float y, float z)
{
    float s = interpolate2D(v_000, v_001, v_010, v_011, x, y);
    float t = interpolate2D(v_100, v_101, v_110, v_111, x, y);
    return interpolate1D(s, t, z);
}

inline Eigen::Vector3f interpolate1DVectors(Eigen::Vector3f v_0, Eigen::Vector3f v_1, float x)
{
    return v_0 * (1 - x) + v_1 * x;
}

inline Eigen::Vector3f interpolate2DVectors(Eigen::Vector3f v_00, Eigen::Vector3f v_01, Eigen::Vector3f v_10, Eigen::Vector3f v_11, float x, float y)
{
    Eigen::Vector3f s = interpolate1DVectors(v_00, v_01, x);
    Eigen::Vector3f t = interpolate1DVectors(v_10, v_11, x);
    return interpolate1DVectors(s, t, y);
}

inline Eigen::Vector3f interpolate3DVectors(Eigen::Vector3f v_000, Eigen::Vector3f v_001, Eigen::Vector3f v_010, Eigen::Vector3f v_011,
                    Eigen::Vector3f v_100, Eigen::Vector3f v_101, Eigen::Vector3f v_110, Eigen::Vector3f v_111,
                    float x, float y, float z)
{
    Eigen::Vector3f s = interpolate2DVectors(v_000, v_001, v_010, v_011, x, y);
    Eigen::Vector3f t = interpolate2DVectors(v_100, v_101, v_110, v_111, x, y);
    return interpolate1DVectors(s, t, z);
}

#endif