#ifndef UTILS_H
#define UTILS_H

#include "Eigen/Dense"
#include <chrono>

class ControlUtils
{
public:
    ControlUtils() {};
    ~ControlUtils() {};

    Eigen::MatrixXf rotMatFromRPY(Eigen::Vector3f rpy)
    {
        double r = rpy[0];
        double p = rpy[1];
        double y = rpy[2];
        Eigen::Matrix3f R_x;
        Eigen::Matrix3f R_y;
        Eigen::Matrix3f R_z;

        R_x << 1,0,0,
            0, std::cos(r), -std::sin(r),
            0, std::sin(r), std::cos(r);

        R_y << std::cos(p) ,0,std::sin(p),
            0, 1, 0,
            -std::sin(p) ,0, std::cos(p);

        R_z << std::cos(y), -std::sin(y), 0,
            std::sin(y), std::cos(y), 0,
            0, 0, 1;

        return R_z * (R_y * R_x);
    };

    Eigen::Vector3f quat_rotate_inverse(Eigen::Vector4f q, Eigen::Vector3f v)
    {
        float q_w = q[0];

        Eigen::Vector3f q_vec(q[1], q[2], q[3]);

        Eigen::Vector3f a = v * (2.0f * q_w * q_w - 1.0f);
        Eigen::Vector3f b = q_vec.cross(v) * q_w * 2.0f;
        Eigen::Vector3f c = q_vec * (q_vec.dot(v)) * 2.0f;

        return a - b + c;
    };

    Eigen::Vector3f get_euler_xyz(const Eigen::Vector4f& q)
    {
        const int qw = 0, qx = 1, qy = 2, qz = 3;

        float sinr_cosp = 2.0f * (q[qw] * q[qx] + q[qy] * q[qz]);
        float cosr_cosp = q[qw] * q[qw] - q[qx] * q[qx] - q[qy] * q[qy] + q[qz] * q[qz];
        float roll = std::atan2(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (q[qw] * q[qy] - q[qz] * q[qx]);
        float pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2.0f, sinp);  // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        float siny_cosp = 2.0f * (q[qw] * q[qz] + q[qx] * q[qy]);
        float cosy_cosp = q[qw] * q[qw] + q[qx] * q[qx] - q[qy] * q[qy] - q[qz] * q[qz];
        float yaw = std::atan2(siny_cosp, cosy_cosp);

        roll = std::fmod(roll + 2 * M_PI, 2 * M_PI);
        pitch = std::fmod(pitch + 2 * M_PI, 2 * M_PI);
        yaw = std::fmod(yaw + 2 * M_PI, 2 * M_PI);

        return Eigen::Vector3f(roll, pitch, yaw);
    };

    class Timer
    {
    public:
        Timer() : beg_(clock_::now()) {}
        void reset() { beg_ = clock_::now(); }
        double elapsed() const {
            return std::chrono::duration_cast<second_>
                (clock_::now() - beg_).count(); }

    private:
        typedef std::chrono::high_resolution_clock clock_;
        typedef std::chrono::duration<double, std::ratio<1> > second_;
        std::chrono::time_point<clock_> beg_;
    };
};



#endif


