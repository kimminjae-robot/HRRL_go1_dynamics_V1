#pragma once
#include <vector>
#include <atomic>
#include <time.h>
#include <unistd.h>

#include <mujoco/mujoco.h>
#include <pthread.h>
#include <string.h>

#include "SharedMemory.hpp"
#include <cassert>

#include <sstream> 
#include <iomanip>
class Daemon{
public:
    Daemon(mjModel* model, mjData* data, SharedMemory* shared_memory);
    void start();
    void join();

private:
    void* loop();  
    static void* thread_function_wrapper(void*);
    mjModel* model_;
    mjData* data_;
    SharedMemory* shared_memory_;
    pthread_t thread_;
    std::atomic<bool> running_;

    int id_gyro_, id_acc_, id_Led_, id_body_;
};