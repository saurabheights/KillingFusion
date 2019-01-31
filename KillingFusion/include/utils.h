#ifndef UTILS_H
#define UTILS_H

// ToDo: Use template.

inline double interpolate1D(double v_0, double v_1, double x)
{
    return v_0 * (1 - x) + v_1 * x;
}

inline double interpolate2D(double v_00, double v_01, double v_10, double v_11, double x, double y)
{
    double s = interpolate1D(v_00, v_01, x);
    double t = interpolate1D(v_10, v_11, x);
    return interpolate1D(s, t, y);
}

inline double interpolate3D(double v_000, double v_001, double v_010, double v_011,
                    double v_100, double v_101, double v_110, double v_111,
                    double x, double y, double z)
{
    double s = interpolate2D(v_000, v_001, v_010, v_011, x, y);
    double t = interpolate2D(v_100, v_101, v_110, v_111, x, y);
    return interpolate1D(s, t, z);
}

inline Eigen::Vector3d interpolate1DVectors(Eigen::Vector3d v_0, Eigen::Vector3d v_1, double x)
{
    return v_0 * (1 - x) + v_1 * x;
}

inline Eigen::Vector3d interpolate2DVectors(Eigen::Vector3d v_00, Eigen::Vector3d v_01, Eigen::Vector3d v_10, Eigen::Vector3d v_11, double x, double y)
{
    Eigen::Vector3d s = interpolate1DVectors(v_00, v_01, x);
    Eigen::Vector3d t = interpolate1DVectors(v_10, v_11, x);
    return interpolate1DVectors(s, t, y);
}

inline Eigen::Vector3d interpolate3DVectors(Eigen::Vector3d v_000, Eigen::Vector3d v_001, Eigen::Vector3d v_010, Eigen::Vector3d v_011,
                    Eigen::Vector3d v_100, Eigen::Vector3d v_101, Eigen::Vector3d v_110, Eigen::Vector3d v_111,
                    double x, double y, double z)
{
    Eigen::Vector3d s = interpolate2DVectors(v_000, v_001, v_010, v_011, x, y);
    Eigen::Vector3d t = interpolate2DVectors(v_100, v_101, v_110, v_111, x, y);
    return interpolate1DVectors(s, t, z);
}

#endif