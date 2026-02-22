#pragma once

#define PI_ 3.141592
#define RAD2DEG 180 / PI_
#define DEG2RAD PI_ / 180

#define UP 1
#define DOWN 0

#define ON   1
#define OFF  0
#define Gravity 9.81


// RBQ10
// #define LEG_0 0.06
// #define LEG_1 0.4
// #define LEG_2 0.4

// #define ROBOT_M 41.1
// #define ROBOT_WIDTH 0.945
// #define ROBOT_PSIDE 0.440 
// #define ROBOT_STAND_HEIGHT 0.6
// #define ROBOT_SWING_HEIGHT 0.2

// #define Trot_Cycle_Length 1.0
// #define Walk_Cycle_Length 2.0
// #define FastTrot_Cycle_Length 0.8

#define LEG_0 0.05
#define LEG_1 0.215
#define LEG_2 0.215

#define ROBOT_M 15.3
#define ROBOT_WIDTH 0.4           
#define ROBOT_PSIDE 0.322         
#define ROBOT_STAND_HEIGHT 0.28
#define ROBOT_SWING_HEIGHT 0.1

#define Trot_Cycle_Length 0.7
#define Walk_Cycle_Length 1.2
#define FastTrot_Cycle_Length 0.5
#define FlyingTrot_Cycle_Length 0.4

#define PI 3.14159265358979323846

enum class ControlMode {
    STANBY,     // 초기 상태 또는 대기
    RL,         // 강화학습 알고리즘
    WBC,        // 동역학 알고리즘
};

enum Foot {
    _FL_,     
    _FR_,  
    _RL_,   
    _RR_=3    
};

enum AXIS {
    _X_,
    _Y_,
    _Z_ = 2
};


