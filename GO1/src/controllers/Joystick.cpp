#include <Joystick.hpp>

Joystick::Joystick(const char* device_id, SharedMemory* shared_memory)
:
    device_id_(device_id),
    shared_memory_(shared_memory)
    {}

void Joystick::initialize()
{
    runnning_ = false;

    speed1_ = 0;
    speed2_ = 0; 
    speed3_ = 0;

    js_ = open(device_id_, O_RDONLY | O_NONBLOCK);
    if (js_ == -1) {
        std::cerr << "Failed to open joystick device: " << device_id_ << std::endl;
    }
    std::cout << "Joystick device opened: " << device_id_ << std::endl;
}

void Joystick::start()
{
    runnning_ = true;
    pthread_create (&thread_, NULL, Joystick::thread_function_wrapper, this);
}

float Joystick::mapAxisValue_(int value) 
{
    return static_cast<float>(value) / 32767.0f;
}

void* Joystick::thread_function_wrapper(void* context)
{
    return static_cast<Joystick*>(context)->Joystick::loop();
}

void* Joystick::loop()
{
    while(runnning_.load()) 
    {
        int bytes = read(js_, &event_, sizeof(event_));
        if (bytes > 0) 
        {
            if (event_.number == 1 ) 
            {
            speed1_ = -mapAxisValue_(event_.value)* 1.;
            }
            if (event_.number == 0 ) 
            {
            speed2_ = -mapAxisValue_(event_.value)* 1.;
            }
            if (event_.number == 3) 
            {
            speed3_ = -mapAxisValue_(event_.value)* 1.;
            }
            shared_memory_->lin_vel_target[0] = speed1_;
            shared_memory_->lin_vel_target[1] = speed2_;
            shared_memory_->ang_vel_target[2] = speed3_;
            // std::cout << speed1_ << std::endl;
        } 
        else if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) 
        {

        } else 
        {
            std::cerr << "Error reading joystick event." << std::endl;
            break;
        }
    }
    return nullptr;
}