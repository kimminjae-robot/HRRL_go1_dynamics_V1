#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip> 
#include <algorithm> 
#include <SharedMemory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <memory>

#include "enum.h"

#include "Dynamics/Algorithm/PostureStabilizer.h" 

#include "Dynamics/Calculator/Kinematics.h"
#include "Dynamics/Calculator/GeneralCalculator.h"
#include "Dynamics/Calculator/RBDL_Tool.h"

#include "Dynamics/Estimator/SlipDetector.h"
#include "Dynamics/Estimator/KalmanFilter.h"
#include "Dynamics/Estimator/KalmanFilterFixed.h"
#include "Dynamics/Estimator/SlopeEstimator.h"

#include "Dynamics/DataStremer/TCPServer.h"
#include "Dynamics/Utils/DynamicsUtil.h"
#include "Dynamics/Utils/TimeLog.h"

class DynamicsController{
public:
    DynamicsController(double Control_frequency, SharedMemory* shared_memory);
    ~DynamicsController();
    void Initialize();  
        void InitKalmanBV() ;
        void InitKalmanDOB() ;

    void MainLoop();
        
        void PreProcessing();
            void GetKinematics();
            void GetDynamics();
            void GetJacobian();

        void StandUpMotion(double now);

        void MidProcessing();
            void StateEstimator();
                Eigen::Vector3d CalcKalmanBV();
                Eigen::VectorXd CalcKalmanDoB();
                // void EstimateSlope(); 
                Eigen::Vector4d CalcSlipProb(bool SW[4], Eigen::MatrixXd& FV);

            void SetReferencePosture();
            void GaitSelector( bool WALKING_FLAG_, int Gait);
            void SwingLegController();
                Eigen::Matrix<double,2,4> CalcRaibertHeuristic(const Eigen::Vector3d& RefVel, const Eigen::Vector3d& ActVel,  double omega_cmd);
                Eigen::Vector4d CalcCapturePoint( double Act_Com_y_vel, double CP_ratio);
                Eigen::Matrix<double,3,4> CalcQuinticSwingTrajectory(Eigen::Vector4d& RefX, Eigen::Vector4d& RefY, double SwingPhase[4]);
                void CalcSwingForceControl(bool mode);
                    Eigen::Vector3d CalcTaskSpaceForce(Eigen::Vector3d& Ref_Pos, Eigen::Vector3d& Act_Pos, Eigen::Vector3d& Ref_Vel, Eigen::Vector3d& Act_Vel);
                    Eigen::Vector3d CalcTaskSpaceCTC( const Eigen::Vector3d& x_d, const Eigen::Vector3d& dx_d, const Eigen::Vector3d& ddx_d, const Eigen::Vector3d& x, const Eigen::Vector3d& dx,    const Eigen::Vector3d& q_dot_,const Eigen::Matrix3d& J, const Eigen::Matrix3d& J_dot_,const Eigen::Matrix3d& Mq, const Eigen::Vector3d& CG  );
            void StanceLegQPController();

        void PostProcessing();
            void applyVirtualDriveDamping(Eigen::Vector3d& JointDamping);
            void TCPDataStreamer();

private:
    // class
    SharedMemory*           shared_memory_;
    PostureStablizer*       PS_; 
    RBDL_Tool               RBDL_;
    Kinematics              Kine_;
    GeneralCalculator       GC_;
    TCPServer               tcp_server_;
    // SlopeEstimator          slope_est_;
    // SlopeEstimator::Output  slope_out_;
    DynamicsUtil            Utils_;
    // KalmanFilter            BodyVelocity, DoB;
    KalmanFilter             DoB;
    KalmanFilterFixed<18, 3, 24> BodyVelocity;
    SlipDetectorBayes       SD_;

    // variables
    double Looptime_ = 0.0;
    double ct = 0.0;
    double control_frequency_ = 500.0; // Hz
    double dt = 1/control_frequency_; // s
    bool Walking_FLAG = false;
    double SensorLPFHz = 5.0; // 센서 데이터 로우패스 필터 주파수

    Eigen::Matrix3d R_Mtrx      = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d R_Mtrx_Trs  = Eigen::Matrix3d::Identity(3,3);

    Eigen::VectorXd q_state     = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd dq_state    = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd Zero6       = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_          = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq_         = Eigen::VectorXd::Zero(12);
    Eigen::Vector3d Euler       = Eigen::Vector3d::Zero(3); // Roll, Pitch, Yaw
    Eigen::Vector3d IMU_linacc  = Eigen::Vector3d::Zero(3); 
    Eigen::Vector3d IMU_angvel  = Eigen::Vector3d::Zero(3);
    Eigen::MatrixXd foot_pos    = Eigen::MatrixXd::Zero(3, 4); // X, Y, Z for each leg
    Eigen::MatrixXd ref_foot_pos= Eigen::MatrixXd::Zero(3, 4); // X, Y, Z for each leg
    Eigen::MatrixXd foot_vel    = Eigen::MatrixXd::Zero(3, 4); // X, Y, Z for each leg
    Eigen::VectorXd Disturbance = Eigen::VectorXd::Zero(6); // 외란 추정 벡터
    Eigen::Matrix3d I3          = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d Z3          = Eigen::Matrix3d::Zero(3,3);
    Eigen::Vector4d slip_Prob   = Eigen::Vector4d::Zero();
    
    bool is_swing[4] = {false, false, false, false}; // FL, FR, RL, RR
    bool WALKING_FLAG = false;
    bool SWING_FLAG = false; 
    int Gait = 0;
    Eigen::Vector4d CP_Dist_Upper = Eigen::Vector4d::Zero();  
    Eigen::Vector4d CP_Dist_Lower = Eigen::Vector4d::Zero();  

    Eigen::Matrix<double,2,4> RaibertOffset = Eigen::Matrix<double,2,4>::Zero();
    Eigen::Vector4d CapturePointOffset         = Eigen::Vector4d::Zero();

    Eigen::Vector4d SwingStartX     = Eigen::Vector4d::Zero(); 
    Eigen::Vector4d SwingStartY     = Eigen::Vector4d::Zero(); 
    Eigen::Vector4d SwingStartZ     = Eigen::Vector4d::Zero(); 

    Eigen::VectorXd WBC_Final_Torque = Eigen::VectorXd::Zero(12); // 최종 토크
    double old_x = 0.0;  
    double old_y = 0.0;  
    double old_z = 0.0; // 이전 Z 위치 (Z 속도 계산을 위한 변수)
    double Mu = 0.25; // 마찰 계수

    struct Risk{
        bool FLAG = false;
        double slip_prob[4] = {0.0, 0.0, 0.0, 0.0};
        Eigen::Vector3d DistForce = Eigen::Vector3d::Zero(3); // 외란 힘
        Eigen::Vector3d DistMoment = Eigen::Vector3d::Zero(3); // 외란 모멘트
        Eigen::Vector3d imu_rpy_ = Eigen::Vector3d::Zero(3); // IMU Roll, Pitch, Yaw
        double desired_theta_R = 0.0; // 원하는 Roll 각도
        double desired_theta_P = 0.0; // 원하는 Pitch 각도
    }Risk_;

    struct StandupTask{
        Eigen::VectorXd RefPose  = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd InitPose = Eigen::VectorXd::Zero(12); 
        Eigen::VectorXd InitVel  = Eigen::VectorXd::Zero(12); 
        Eigen::VectorXd Kp       = 60.0 * Eigen::VectorXd::Ones(12);
        Eigen::VectorXd Kd       =  2.0 * Eigen::VectorXd::Ones(12);
        double         startTime = 0.0;      // 시작 시각 (초)
        const double   duration  = 2.0;      // 보간 전 시간 (초)
        bool           started   = false;    // 시작 플래그
        bool           finished  = false;    // 완료 플래그
    }StandUp;

    struct Jacobian {
        struct Foot{
            Eigen::Matrix3d J       = Eigen::Matrix3d::Zero(3, 3);
            Eigen::Matrix3d J_dot   = Eigen::Matrix3d::Zero(3, 3);
            Eigen::Matrix3d J_Trs   = Eigen::Matrix3d::Zero(3, 3);
        }FL, FR, RL, RR;
        struct Body{
            Eigen::MatrixXd Whole       = Eigen::MatrixXd::Zero(18, 18);
            Eigen::MatrixXd Whole_Trs   = Eigen::MatrixXd::Zero(18, 18);
            Eigen::MatrixXd Whole_Dot   = Eigen::MatrixXd::Zero(18, 18);
            Eigen::MatrixXd Base        = Eigen::MatrixXd::Zero(6,  6);
        }Body;
        Eigen::MatrixXd LEGs         = Eigen::MatrixXd::Zero(12, 12);

    }J_;

    struct Dynamics{
        struct Inertia{
            Eigen::MatrixXd Body    = Eigen::MatrixXd::Zero(6, 18);
            Eigen::MatrixXd Leg     = Eigen::MatrixXd::Zero(12, 18);
            struct LegInertia{
                Eigen::Matrix3d M3   = Eigen::Matrix3d::Zero(3, 3);
            }FL_, FR_, RL_, RR_;
            Eigen::Matrix3d BodyInertiaRotation = Eigen::Matrix3d::Zero(3, 3); // Body Inertia Rotation Matrix
        }M_;
        struct Nonlinear{
            Eigen::VectorXd Body    = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd Leg     = Eigen::VectorXd::Zero(12);
            struct LegNonlinear{
                Eigen::VectorXd CG3   = Eigen::VectorXd::Zero(3);
            }FL_, FR_, RL_, RR_;
        }CG_;
        struct Force{
            Eigen::VectorXd Body    = Eigen::VectorXd::Zero(6); // X, Y, Z, Roll, Pitch, Yaw
            Eigen::VectorXd Leg     = Eigen::VectorXd::Zero(12); // FL, FR, RL, RR
            struct LegForce{
                Eigen::Vector3d EEForce   = Eigen::Vector3d::Zero(3); // X, Y, Z
                Eigen::Vector3d QPLambda  = Eigen::Vector3d::Zero(3); // X, Y, Z
            }FL_, FR_, RL_, RR_;
        }Force;
        struct Torque{
            Eigen::VectorXd Body    = Eigen::VectorXd::Zero(6); // X, Y, Z, Roll, Pitch, Yaw
            Eigen::VectorXd Leg     = Eigen::VectorXd::Zero(12); // FL, FR, RL, RR
            struct LegTorque{
                Eigen::Vector3d Leg = Eigen::Vector3d::Zero(3); // Hip roll, Hip pitch, Knee Pitch
                Eigen::Vector3d SwingTorque = Eigen::Vector3d::Zero(3); // Hip roll, Hip pitch, Knee Pitch
            } FL_, FR_, RL_, RR_;
        }Torque;
    }Dyn_;
    
    struct State{
        struct CoM{
            Eigen::VectorXd pos = Eigen::VectorXd::Zero(6); // X, Y, Z, Roll, Pitch, Yaw
            Eigen::VectorXd vel = Eigen::VectorXd::Zero(6); // X, Y, Z, Roll, Pitch, Yaw
        }CoM_;
        struct Foot{
            Eigen::Vector3d pos = Eigen::VectorXd::Zero(3); // X, Y, Z
            Eigen::Vector3d vel = Eigen::VectorXd::Zero(3); // X, Y, Z
            Eigen::Vector3d acc = Eigen::VectorXd::Zero(3); // X, Y, Z
        }FL_, FR_, RL_, RR_;
    }Ref, Act, World;

    struct GaitPattern {
        int    ct            = 0;
        double hz            = 500.0;   // 호출 주파수 (Hz) 1000/500
        double time          = 0.0;     // Pattern_count / Pattern_hz

        bool   prev_walking_flag        = false;
        int    prev_gait_mode           = 0;      
        
        bool   first_cycle              = true;
        double cycle_start_time         = 0.0;
        double cycle_length             = 0.0;     // 전체 한 사이클 길이
        double time_sw                  = 0.0;     // swing phase 길이
        double time_st                  = 0.0;     // stance phase 길이

        bool    SW_FLAG[4]          = {DOWN, DOWN, DOWN, DOWN}; // FL, FR, RL, RR
        bool    old_SW_FLAG[4]      = {DOWN, DOWN, DOWN, DOWN}; // FL, FR, RL, RR
        double  Sw_timing[2][4]     = {{0.0}};
        double  SwingPhase[4]       = {0.0, 0.0, 0.0, 0.0}; // 스윙 페이즈 중 지나간 기간 (FL, FR, RL, RR)
        double  walk_pattern_ratio[4] = { 0.0, 0.5, 0.75, 0.25  }; 
        double  trot_pattern_ratio[4] = { 0.0, 0.5, 0.5, 0.0  }; 

        bool   req_walking_flag = false;
        int    req_gait_mode    = 0;

        double stanceXtraj[4] = {ROBOT_WIDTH/2, ROBOT_WIDTH/2, -ROBOT_WIDTH/2, -ROBOT_WIDTH/2}; // 스윙 시작 시점의 발 위치 (X, Y, Z 축)
        double stanceYtraj[4] = {ROBOT_PSIDE/2, -ROBOT_PSIDE/2, ROBOT_PSIDE/2, -ROBOT_PSIDE/2}; // 스윙 시작 시점
        double stanceZtraj[4] = {-ROBOT_STAND_HEIGHT,-ROBOT_STAND_HEIGHT,-ROBOT_STAND_HEIGHT,-ROBOT_STAND_HEIGHT }; // 스윙 시작 시점의 발 위치 (X, Y, Z 축)

    }Pattern_;

    struct StateModel{
        Eigen::MatrixXd A ;
        Eigen::MatrixXd B ;
        Eigen::MatrixXd C ;
        Eigen::VectorXd x ; 
        Eigen::VectorXd u ;
        Eigen::VectorXd y ;
        Eigen::MatrixXd Q ;
        Eigen::MatrixXd R ;
        Eigen::MatrixXd P ;
        Eigen::MatrixXd K ;
        Eigen::MatrixXd I ;
        int stateDim ;
        int inputDim ;
        int outputDim ;
    }Vel_model_, SRB_model_;

    struct Gain { double kp, kd; };

    using ModeGains = std::map<std::string, Gain>;
    struct RobotConfig {
        std::string name;
        std::map<std::string, ModeGains> wbc; 
        Gain swing_taskspace;  
        Gain swing_ctc;       
    }robot_config_;

    struct QPGains {
        struct CoM{
            Eigen::Vector3d KP = Eigen::Vector3d::Zero(); 
            Eigen::Vector3d KD = Eigen::Vector3d::Zero(); 

        }CoM_;
        struct RPY{
            Eigen::Vector3d KP = Eigen::Vector3d::Zero(); 
            Eigen::Vector3d KD = Eigen::Vector3d::Zero(); 
        }RPY_;
    }QP_;

    struct Slope {
        Eigen::Vector3d ground_normal_w_ = Eigen::Vector3d(0,0,1);
        double ground_roll_  = 0.0;
        double ground_pitch_ = 0.0;
    }Slope_;

    RobotConfig loadConfig(const std::string& filename) ;
    void applyGainsToQP(bool walking_flag);
    
};
