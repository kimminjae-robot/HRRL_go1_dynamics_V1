#include "Daemon.hpp"
#include <iostream>

Daemon::Daemon(mjModel* model, mjData* data, SharedMemory* shared_memory)
    :
        model_(model),
        data_(data),
        shared_memory_(shared_memory),
        running_(false)
        {
            id_gyro_ = mj_name2id(model_, mjOBJ_SENSOR, "imu_gyro");
            id_acc_  = mj_name2id(model_, mjOBJ_SENSOR, "imu_acc");
        }
        
void Daemon::start()
{
    running_ = true;
    int flag = pthread_create(&thread_, NULL, Daemon::thread_function_wrapper, this);
    if (flag != 0) {
        std::cerr << "Failed to create thread: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE); // 또는 예외 던지기
    }
    
}
void Daemon::join()
{
    pthread_join(thread_, nullptr);
}

void* Daemon::thread_function_wrapper(void* context)
{
    return static_cast<Daemon*>(context)->Daemon::loop();
}

void* Daemon::loop()
{

    float frequency = 500.0f;
    const long PERIOD_US = long((1.0f/frequency)*1e6);
    struct timespec TIME_NEXT;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    std::cout << "Daemon Loop Start" << std::endl;
    using Clock = std::chrono::high_resolution_clock;

    int loop_count = 0;
    double avg_period_ms = 0.0;

    while (running_.load())
    {
        auto tic = Clock::now();
        for (int i = 0; i < 3; i++){
            shared_memory_->gyro[i]  = data_->qvel[i+3];
        }
        // --- qpos, qvel 복사 (기존) ---
        for (int i = 0; i < 12; i++)
        {
            shared_memory_->q[i]  = data_->qpos[i+7];
            shared_memory_->qd[i] = data_->qvel[i+6];
        }
        for (int i = 0; i < 4; i++)
            shared_memory_->quat[i] = data_->qpos[i+3];

        // --- IMU 읽기 & 출력 ---
        {
            int adr_gyro = model_->sensor_adr[id_gyro_];
            int adr_acc  = model_->sensor_adr[id_acc_];
            int dim_gyro = model_->sensor_dim[id_gyro_];  // 보통 3
            int dim_acc  = model_->sensor_dim[id_acc_];   // 보통 3

            assert(adr_gyro >= 0 && adr_gyro + dim_gyro <= model_->nsensordata);
            assert(adr_acc  >= 0 && adr_acc  + dim_acc  <= model_->nsensordata);

            double imu_gyro[3];
            double imu_acc [3];

            for (int i = 0; i < dim_gyro; i++) {
                imu_gyro[i] = data_->sensordata[adr_gyro + i];
            }
            for (int i = 0; i < dim_acc; i++) {
                imu_acc[i]  = data_->sensordata[adr_acc  + i];
            }
            for (int i = 0; i < 3; i++){
                shared_memory_->IMU[i] = imu_gyro[i];
                shared_memory_->IMU[i+3] = imu_acc[i];
            }
        }

        for (int i = 0; i < 12; i++)
            data_->ctrl[i] = shared_memory_->Dyn_torque[i];

        // --- 주기 맞추기 (기존) ---
        TIME_NEXT.tv_nsec += PERIOD_US * 1000;
        while (TIME_NEXT.tv_nsec >= 1000000000L) {
            TIME_NEXT.tv_nsec -= 1000000000L;
            TIME_NEXT.tv_sec++;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);

        auto toc = Clock::now();
        avg_period_ms += std::chrono::duration<double, std::milli>(toc - tic).count();
        loop_count++;
        if (loop_count >= int(frequency)) {
            double avg_hz = 1000.0 / (avg_period_ms / loop_count);
            std::cout << "[Daemon] Avg period: "
                      << (avg_period_ms/loop_count) << " ms ("
                      << avg_hz << " Hz)\n";
            loop_count = 0;
            avg_period_ms = 0.0;
        }
    }

    return nullptr;
}

