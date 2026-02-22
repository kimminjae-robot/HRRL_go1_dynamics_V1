#include "TaskController.hpp"

#include <iostream>

TaskController::TaskController(double control_frequency, SharedMemory* shared_memory)
    :
    control_frequency_(control_frequency),
    shared_memory_(shared_memory),
    running_(false), 
    DC_(nullptr)
    {}

void TaskController::initialize()
{
    if (joint_idx_style == "raisim" || joint_idx_style == "rainbow")
    {
        home_idx_conversion = {9, 10, 11, 6, 7, 8, 3, 4, 5, 0, 1, 2};
        input_joint_idx_conversion = {9, 6, 3, 0, 10, 7, 4, 1, 11 ,8, 5, 2};
        output_joint_idx_conversion = {3, 7, 11, 2, 6, 10, 1, 5, 9 ,0, 4, 8};
    } else if ( joint_idx_style == "unitree"){
        home_idx_conversion = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8 };
        input_joint_idx_conversion = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8 };
        output_joint_idx_conversion = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8 };
    }
    default_joint_angle_ = {
        0.f, 0.9f, -1.8f,
        0.f, 0.9f, -1.8f,
        0.f, 0.9f, -1.8f,
        0.f, 0.9f, -1.8f
    };

    YAML::Node runner_config = YAML::LoadFile("../config/runner_config.yaml");
    runner_ = new OnnxRunner("policy_1.onnx", runner_config);

    state_ = TASK_IDLE;
    DC_    = new DynamicsController(500, shared_memory_);
    DC_->Initialize();
}
void TaskController::start()
{
    running_ = true;
    pthread_create(&thread_, NULL, TaskController::thread_function_wrapper, this);
}

void TaskController::join()
{
    pthread_join(thread_, nullptr);
}

void* TaskController::thread_function_wrapper(void* context)
{
    return static_cast<TaskController*>(context)->TaskController::loop();
}

void* TaskController::loop()
{
    float frequency = 500.0f;   // Control frequency in Hz
    float sleepMS_ = 1000.0f * (1/frequency);
    const long PERIOD_US = sleepMS_ * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;
    
    std::cout << "TaskController Loop Start" << std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    int cnt = 0;

    while(running_.load())
    {   
        cnt++;
        if (cnt >= 100)
        {
            state_ = TASK_WBC;
        }
        switch(state_)
        {
            case TASK_IDLE:
            {
                for (int i=0;i<12;i++)
                {
                    shared_memory_->q_target[i] = default_joint_angle_[i];
                }
                break;
            }
            case TASK_RL:
            {
                Eigen::Vector3f gyro_vec;
                gyro_vec << static_cast<float>(shared_memory_->gyro[0]),
                            static_cast<float>(shared_memory_->gyro[1]),
                            static_cast<float>(shared_memory_->gyro[2]);
                runner_->set_base_angular_velocity(gyro_vec);

                //quat =(w, x, y, z)
                Eigen::Quaternionf eigen_quat = Eigen::Quaternionf(shared_memory_->quat[0],
                                                shared_memory_->quat[1],
                                                shared_memory_->quat[2],
                                                shared_memory_->quat[3]);
                runner_->set_quaternion(eigen_quat);

                runner_->set_commands(  shared_memory_->lin_vel_target[0], 
                                        shared_memory_->lin_vel_target[1], 
                                        shared_memory_->ang_vel_target[2]);

                float dof_pos[12], dof_vel[12];
                for (int i = 0; i < 12; i++)
                {
                    dof_pos[i] = shared_memory_->q[i];
                    dof_vel[i] = shared_memory_->qd[i];
                }
                runner_->set_joints(dof_pos, dof_vel);

                // Compute Policy
                runner_->compute_policy();
                for (int i = 0; i < 12; i++)
                {
                    shared_memory_->q_target[i] = (default_joint_angle_[i] + runner_->get_action()[i]);
                }
                // for (int i = 0; i < 6; i++)
                // {
                //     shared_memory_->q_target[i+6] = (default_joint_angle_[i] + runner_->get_action()[i]);
                //     shared_memory_->q_target[i] = (default_joint_angle_[i+6] + runner_->get_action()[i+6]);
                // }
                // std::cout << "RL_ MainLoop" << std::endl;

                break;
            }
            case TASK_WBC:
            {
                DC_->MainLoop();
                // state_ = TASK_RL;
                break;
            }
            
            default:
            {
                break;
            }
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);

        TIME_NEXT.tv_nsec += PERIOD_US * 1000;
        while (TIME_NEXT.tv_nsec >= 1e9) {
            TIME_NEXT.tv_nsec -= 1e9;
            TIME_NEXT.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if ((TIME_NOW.tv_sec > TIME_NEXT.tv_sec) || (TIME_NOW.tv_sec == TIME_NEXT.tv_sec && TIME_NOW.tv_nsec > TIME_NEXT.tv_nsec)) 
        {
            std::cout << "!!Deadline Misssed!!" << std::endl;
            clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

        }

    }
    return nullptr;
}
