#pragma once
#include <vector>
#include <thread>
#include <mutex>

struct SharedMemory 
{
    std::vector<double> q;
    std::vector<double> qd;

    std::vector<double> q_target;
    std::vector<double> torque_target;

    std::vector<double> quat;
    std::vector<double> gyro;

    std::vector<double> lin_vel_target;
    std::vector<double> ang_vel_target;

    std::vector<double> IMU;
    std::vector<double> Dyn_torque;

    bool LED; 
    bool ControllerMode ; // 0: RL, 1: WBC

    std::mutex mtx;

    SharedMemory(int joint_count)
    : q(joint_count, 0.0),
      qd(joint_count, 0.0),
      q_target(joint_count, 0.0),
      torque_target(joint_count, 0.0),
      quat(4, 0.0),
      gyro(3, 0.0),
      lin_vel_target(3, 0.0),
      ang_vel_target(3, 0.0), 
      IMU(6, 0.0),  // 3 for gyro, 3 for accelerometer
      Dyn_torque(joint_count, 0.0),  // Dynamic torque for each joint
      LED(false),
      ControllerMode(0)  // Default to RL mode
    {}

};