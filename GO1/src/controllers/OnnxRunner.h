#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>


class OnnxRunner
{
public:
    OnnxRunner(std::string file_name, const YAML::Node& config);
    ~OnnxRunner();

    void set_commands(float command_x, float command_y, float command_yaw);
    void set_joints(float pos[], float vel[]);
    void set_base_angular_velocity(Eigen::Vector3f base_ang_vel);
    void set_quaternion(Eigen::Quaternionf quat);

    void reset_observations();

    void compute_policy();

    std::vector<float> get_action() { return scaled_actions; }

private:


    // FUNCTION
    void parsing_yaml_(const YAML::Node& config);
    void initialize_policy_();
    std::vector<float> compute_observation_();

    // VARIABLES
    // -------------for onnx-----------------

    std::string file_path_, file_name_;
    std::string policy_path;

    Ort::Env env;
    Ort::Session* session;
    Ort::SessionOptions session_options;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::string> input_names_str; 
    std::vector<std::string> output_names_str;
    //------------------------------

    int num_joint_;

    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;
    float action_clip_;

    Eigen::Quaternionf quat_;
    Eigen::Vector3f gravity_vec_;
    Eigen::Vector3f base_ang_vel_;
    Eigen::Vector3f commands_;

    std::vector<float> dof_pos;
    std::vector<float> dof_vel;
    std::vector<float> default_joint_angle_;
    std::vector<float> delta_dof_pos;
    std::vector<float> actions;
    std::vector<float> scaled_actions;

    std::vector<int> input_joint_idx_conversion_, output_joint_idx_conversion_;
    std::string joint_idx_style_;

};

