#pragma once

#include <atomic>

#include <pthread.h>
#include <yaml-cpp/yaml.h>

#include <OnnxRunner.h>
#include <SharedMemory.hpp>

#include "DynamicsController.hpp" 

class TaskController{
public:
    TaskController(double Control_frequency, SharedMemory* shared_memory);
    void initialize();
    void start();
    void join();

private:
    static void* thread_function_wrapper(void*);
    void* loop();

    enum TaskMode
    {
        TASK_IDLE = 0,
        TASK_HOMMING,
        TASK_RL,
        TASK_WBC,
    };

    SharedMemory* shared_memory_;
    OnnxRunner* runner_;
    pthread_t thread_;
    DynamicsController* DC_;
    double control_frequency_;
    int state_;
    std::atomic<bool> running_;

    std::vector<float> default_joint_angle_;
    std::vector<double> theta_target;
    std::vector<int> input_joint_idx_conversion, output_joint_idx_conversion, home_idx_conversion;
    // std::string joint_idx_style = "rainbow";
    std::string joint_idx_style = "unitree_conversion";
};

