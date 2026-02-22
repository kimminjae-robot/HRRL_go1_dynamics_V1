#include "OnnxRunner.h"

OnnxRunner::OnnxRunner(std::string file_name, const YAML::Node& config)
{
    file_name_ = file_name;
    gravity_vec_ << 0, 0, -1;

    parsing_yaml_(config);

    reset_observations();

    initialize_policy_();

    std::cout << "[OnnxRunner] OnnxRunner initialized" << std::endl;
}
OnnxRunner::~OnnxRunner()
{
    delete session;
}

void OnnxRunner::parsing_yaml_(const YAML::Node& config)
{
    file_path_ = config["file_path"].as<std::string>();

    num_joint_ = config["num_joint"].as<int>();

    dof_pos.resize(num_joint_);
    dof_vel.resize(num_joint_);
    default_joint_angle_.resize(num_joint_);
    delta_dof_pos.resize(num_joint_);
    actions.resize(num_joint_);
    scaled_actions.resize(num_joint_);

    joint_idx_style_ = config["joint_idx_style"].as<std::string>();
    std::string group_name = "joint_idx_conversion_" + joint_idx_style_;
    input_joint_idx_conversion_ = config[group_name]["input"].as<std::vector<int>>();
    output_joint_idx_conversion_ = config[group_name]["output"].as<std::vector<int>>();
    std::cout << "[OnnxRunner -- ] Joint Index Style: " << joint_idx_style_ << std::endl;
    std::vector<float> default_joint_angle_original = config["default_joint_angle"].as<std::vector<float>>();
    
    int i=0;
    for (int idx : input_joint_idx_conversion_)
    {
        default_joint_angle_[i] = default_joint_angle_original[idx];
        i++;
    }

    dof_pos_scale = config["scale"]["dof_pos"].as<float>();
    dof_vel_scale = config["scale"]["dof_vel"].as<float>();
    ang_vel_scale = config["scale"]["ang_vel"].as<float>();
    lin_vel_scale = config["scale"]["lin_vel"].as<float>();
    action_scale = config["scale"]["action_scale"].as<float>();
    action_clip_ = config["action_clip"].as<float>();

}

void OnnxRunner::initialize_policy_()
{
    policy_path = file_path_ + file_name_;

    env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "OnnxRunner");
    session_options = Ort::SessionOptions();
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session = new Ort::Session(env, policy_path.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;

    size_t num_inputs = session->GetInputCount();
    for (size_t i = 0; i < num_inputs; i++) {
        Ort::AllocatedStringPtr input_name_ptr = session->GetInputNameAllocated(i, allocator);
        input_names_str.push_back(std::string(input_name_ptr.get()));
        input_names.push_back(input_names_str.back().c_str());
    }

    size_t num_outputs = session->GetOutputCount();
    for (size_t i = 0; i < num_outputs; i++) {
        Ort::AllocatedStringPtr output_name_ptr = session->GetOutputNameAllocated(i, allocator);
        output_names_str.push_back(std::string(output_name_ptr.get()));
        output_names.push_back(output_names_str.back().c_str());
    }

    std::cout << "[OnnxRunner] ONNX environment initialized" << std::endl;
}

void OnnxRunner::set_commands(float command_x, float command_y, float command_yaw)
{
    commands_ << command_x, command_y, command_yaw;
}

void OnnxRunner::set_joints(float pos[], float vel[])
{
    
    std::vector<float> pos_reordered(num_joint_,0.0);
    std::vector<float> vel_reordered(num_joint_,0.0);

    int i=0;
    for (int idx : input_joint_idx_conversion_)
    {
        pos_reordered[i] = pos[idx];
        vel_reordered[i] = vel[idx];
        i++;
    }

    for (int i = 0; i < num_joint_; i++)
    {
        dof_pos[i] = pos_reordered[i];
        delta_dof_pos[i] = dof_pos[i] - default_joint_angle_[i];
        dof_vel[i] = vel_reordered[i];
    }
}

void OnnxRunner::set_base_angular_velocity(Eigen::Vector3f base_ang_vel)
{
    base_ang_vel_ = base_ang_vel;
}

void OnnxRunner::set_quaternion(Eigen::Quaternionf quat)
{
    quat_ = quat;
}

std::vector<float> OnnxRunner::compute_observation_()
{
    std::vector<float> obs_vec;

    Eigen::Matrix3f rotation_matrix = quat_.normalized().Eigen::Quaternionf::toRotationMatrix();
    Eigen::Vector3f projected_gravity = rotation_matrix.transpose() * gravity_vec_;
    for (int i = 0; i < 3; i++)
        obs_vec.push_back(projected_gravity[i]);

    for (int i = 0; i < 3; i++)
        obs_vec.push_back(base_ang_vel_[i] * ang_vel_scale);

    static float commands_scale[] = {lin_vel_scale, lin_vel_scale, ang_vel_scale};
    for (int i = 0; i < 3; i++)
        obs_vec.push_back(commands_[i] * commands_scale[i]);


    for (int i = 0; i < num_joint_; i++)
        obs_vec.push_back(delta_dof_pos[i] * dof_pos_scale);


    for (int i = 0; i < num_joint_; i++)
        obs_vec.push_back(dof_vel[i] * dof_vel_scale);


    // 기존
    for (int i = 0; i < num_joint_; i++)
        // obs_vec.push_back(actions[i]);
        obs_vec.push_back(dof_pos[i]);

    // for (int i = 0; i < 3; i++){
    //     obs_vec.push_back(dof_pos[i+3]);
    // }
        
    // for (int i = 0; i < 3; i++){
    //     obs_vec.push_back(dof_pos[i]);
    // }

    // for (int i = 0; i < 3; i++){
    //     obs_vec.push_back(dof_pos[i+9]);
    // }

    // for (int i = 0; i < 3; i++){
    //     obs_vec.push_back(dof_pos[i+6]);
    // }


    return obs_vec;
}

void OnnxRunner::compute_policy()
{
    std::vector<float> obs_vec = OnnxRunner::compute_observation_();


    std::vector<int64_t> input_shape{1, static_cast<int64_t>(obs_vec.size())};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, obs_vec.data(), obs_vec.size(), input_shape.data(), input_shape.size()
    );

    auto output_tensors = session->Run(
        Ort::RunOptions{nullptr},
        input_names.data(), &input_tensor, 1,
        output_names.data(), 1
    );

    float* output_data = output_tensors[0].GetTensorMutableData<float>();

    float scaled_actions_original[num_joint_];

    for (int i = 0; i < num_joint_; i++)
    {
        actions[i] = std::clamp(output_data[i], -action_clip_, action_clip_);
        scaled_actions_original[i] = actions[i] * action_scale;
    }

    int i=0;
    for (int idx : output_joint_idx_conversion_)
    {
        scaled_actions[i] = scaled_actions_original[idx];
        i++;
    }

}

void OnnxRunner::reset_observations()
{
    for (int i = 0; i < num_joint_; i++)
    {
        dof_pos[i] = 0.f;
        dof_vel[i] = 0.f;
        delta_dof_pos[i] = 0.f;
        actions[i] = 0.f;
        scaled_actions[i] = 0.f;
    }
}
