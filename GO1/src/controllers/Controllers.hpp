#pragma once

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include "JointPositionPDController.hpp"
#include "SharedMemory.hpp"
#include "Daemon.hpp"
#include "TaskController.hpp"
#include "Joystick.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <future>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <mujoco/mujoco.h>
#include "../simulate/glfw_adapter.h"
#include "../simulate/simulate.h"
#include "../simulate/array_safety.h"



struct Controllers
{
  JointPositionPDController* pd_controller;
  SharedMemory* shared_memory;
  Daemon* daemon;
  TaskController* task_controller;
  Joystick* joystick_;
};
