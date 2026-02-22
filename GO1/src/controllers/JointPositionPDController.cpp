#include "JointPositionPDController.hpp"

#include <iostream>

JointPositionPDController::JointPositionPDController(double control_frequency, int num_joint, double kp, double kd)
    :   
        control_frequency_(control_frequency),
        num_joint_(num_joint), 
        kp_(kp),
        kd_(kd),
        q_actual_(num_joint, 0.0),
        qd_actual_(num_joint, 0.0),
        q_target_(num_joint, 0.0)
        {}

void JointPositionPDController::set_joint_actual(const std::vector<double>& q_actual, const std::vector<double>& qd_actual)
{
    for (int i=0; i<num_joint_; i++)
    {
        q_actual_[i] = q_actual[i];
        qd_actual_[i] = qd_actual[i];
    }

}
void JointPositionPDController::set_joint_target(const std::vector<double>& q_target)
{
    for (int i=0; i<num_joint_; i++)
    {
        q_target_[i] = q_target[i];
    }
}

std::vector<double> JointPositionPDController::compute_torque()
{
    std::vector<double> torque(num_joint_, 0.0);

    for(int i=0; i<num_joint_; i++)
    {
        // std::cout << (q_target_[i] - q_actual_[i]) << std::endl;
        torque[i] =  kp_ * (q_target_[i] - q_actual_[i]) - kd_ * qd_actual_[i];
    }
    return torque;
}

double JointPositionPDController::get_control_frequency(){return control_frequency_;}