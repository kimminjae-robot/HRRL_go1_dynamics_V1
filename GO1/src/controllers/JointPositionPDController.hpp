#pragma once
#include <vector>

class JointPositionPDController{
public:
    JointPositionPDController(double control_frequency, int num_joint, double kp, double kd);
    

    void set_joint_actual(const std::vector<double>& q_actual, const std::vector<double>& qd_actual);
    void set_joint_target(const std::vector<double>& q_target);

    std::vector<double> compute_torque();
    
    void step();
    double get_control_frequency();

private:

    int num_joint_;
    double control_frequency_;
    double kp_, kd_;

    int count = 0;

    std::vector<double> q_actual_, qd_actual_;
    std::vector<double> q_target_;
    

};