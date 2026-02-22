#pragma once
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

#include <pthread.h>

#include <SharedMemory.hpp>

#include <iostream>
#include <atomic>

class Joystick
{
public:
    Joystick(const char* device_id, SharedMemory* shared_memory);
    void initialize();
    void start();
    static void* thread_function_wrapper(void*); 
    void* loop();

private:
    float mapAxisValue_(const int value);

    struct js_event event_;

    pthread_t thread_;
    SharedMemory* shared_memory_;

    const char* device_id_;

    int js_;
    float speed1_, speed2_, speed3_;
    std::atomic<bool> runnning_;

};