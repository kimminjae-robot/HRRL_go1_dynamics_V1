#include "DynamicsController.hpp"

DynamicsController::DynamicsController( double control_frequency, SharedMemory* shared_memory):
    control_frequency_(control_frequency),
    shared_memory_(shared_memory),
    PS_(new PostureStablizer()),
    BodyVelocity(),DoB( 12,12,6),  tcp_server_(9000)
    {
        if (!tcp_server_.start()) {
            std::cerr << "TCP 서버 시작 실패 (port=9000)" << std::endl;
        }   
    }
DynamicsController::~DynamicsController()
{
    delete PS_;
}
void DynamicsController::Initialize()
{   
    // RBDL 초기화
    RBDL_.Init_RBDL();
    InitKalmanBV();
    InitKalmanDOB();

    robot_config_ = loadConfig("../config/WBC_config.yaml");

    // ---- 경사 추정기 초기화 ----
    // SlopeEstimator::Config scfg;
    // scfg.ema_alpha = 0.2;                 // 0.1~0.3 추천 (노이즈 많으면 낮게)
    // scfg.two_point_min_separation = 1e-3; // 1mm
    // scfg.enforce_upward_normal = true;
    // slope_est_ = SlopeEstimator(scfg);
    // slope_est_.reset(Eigen::Vector3d::UnitZ());
}
void DynamicsController::InitKalmanBV( )
{   
    Eigen::VectorXd Initial_X(18);
    Initial_X << 0, 				0, 				(-foot_pos(2,0) -foot_pos(2,1) -foot_pos(2, 2) -foot_pos(2, 3))/4, 
                 0, 				0, 				0, 
                 ROBOT_WIDTH/2, 	ROBOT_PSIDE/2, 	0,         
                 ROBOT_WIDTH/2,    -ROBOT_PSIDE/2,	0,     	
                -ROBOT_WIDTH/2,	    ROBOT_PSIDE/2, 	0,     
                -ROBOT_WIDTH/2,    -ROBOT_PSIDE/2,  0; 
    BodyVelocity.setState(Initial_X);

    // P - Tune!
    std::array<double,6> diagP = {100, 100, 0.001, 0.001, 0.001, 0.001};
    Eigen::MatrixXd Initial_P = Eigen::MatrixXd::Zero(18, 18);
    for (int i = 0; i < 6; ++i)Initial_P.block<3,3>(3*i, 3*i) = I3 * diagP[i];
    BodyVelocity.setCov(Initial_P);

    // Q - Tune!
    std::array<double,6> diagQ = {std::pow(0.001,2),std::pow(0.001,1),std::pow(0.001,2),std::pow(0.001,2),std::pow(0.001,2),std::pow(0.001,2)};
    Eigen::MatrixXd Initial_Q = Eigen::MatrixXd::Zero(18, 18);
    for (int i = 0; i < 6; ++i)Initial_Q.block<3,3>(3*i, 3*i) = I3 * diagQ[i];
    BodyVelocity.setQ(Initial_Q);

    // Noise - Tune!
    double o_noise1=0.000002;	        double o_noise2=0.02;
    std::array<double,8> diagR = { o_noise1, o_noise1, o_noise1, o_noise1, o_noise2, o_noise2, o_noise2, o_noise2 };
    Eigen::MatrixXd Initial_R = Eigen::MatrixXd::Zero(24, 24);
    for (int i = 0; i < 8; ++i) {
        Initial_R.block<3,3>(3*i, 3*i) = I3 * diagR[i] * 0.49;
    }
    BodyVelocity.setR(Initial_R);    
}
void DynamicsController::InitKalmanDOB( )
{   
    SRB_model_.stateDim = 12;            
    SRB_model_.inputDim = 12;            
    SRB_model_.outputDim = 6;
    
    // A
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(SRB_model_.stateDim, SRB_model_.stateDim);
    A.block<3, 3>(0, 0) = I3;
    A.block<3, 3>(3, 3) = I3;
    A.block<3, 3>(0, 6) = (dt / ROBOT_M) * I3;
    A.block<3, 3>(3, 9) = dt * I3;  
    A.block<3, 3>(6, 6) = I3;
    A.block<3, 3>(9, 9) = I3;
    DoB.setA( A );

    // B
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(SRB_model_.stateDim, SRB_model_.inputDim);
    Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(12, 12); // 입력 신뢰도 가중치 행렬
    for (int i = 0; i < 4; ++i)
    {
        double trust = 1.0;
        B.block<3, 3>(0, i * 3) = (dt / ROBOT_M) * I3;
        Eigen::Vector3d r_i = foot_pos.col(i);
        Eigen::Matrix3d skew_r_i;
        skew_r_i <<      0, -r_i(2),  r_i(1),
                    r_i(2),      0, -r_i(0),
                   -r_i(1),  r_i(0),      0;
        B.block<3, 3>(3, i * 3) = -dt * skew_r_i;
        W_b.block<3,3>(i*3, i*3) *= trust;
    }
    DoB.setB( B* W_b);  // 신뢰도 가중치 반영된 B

    // C, Q, R 초기화
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(SRB_model_.outputDim, SRB_model_.stateDim);
    C.block<3, 3>(0, 6) = (1.0 / ROBOT_M) * I3; 
    C.block<3, 3>(3, 3) = I3;
    C.block<3, 3>(3, 9) = I3;
    DoB.setC( C );

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(SRB_model_.stateDim, SRB_model_.stateDim);
    Q.block<3,3>(0,0)   = I3 * 1e-3; // v_com
    Q.block<3,3>(3,3)   = I3 * 1e-4; // w_com
    Q.block<3,3>(6,6)   = I3 * 1e-2; // F_ext
    Q.block<3,3>(9,9)   = I3 * 1e-1; // tau_ext
    DoB.setQ( Q );

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(SRB_model_.outputDim, SRB_model_.outputDim);// * pow(0.001, 5);
    R.block<3,3>(0,0) = I3 * pow(0.001, 3); // 1e-3; // a_imu
    R.block<3,3>(3,3) = I3 * pow(0.001, 3); // 1e-4; // w_imu
    DoB.setR( R );

    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(SRB_model_.stateDim);
    DoB.setx0( x_hat );

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(SRB_model_.stateDim, SRB_model_.stateDim) * 1.0;
    DoB.setP0( P );

}
void DynamicsController::MainLoop()
{
    PreProcessing();
    MidProcessing();
    PostProcessing();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynamicsController::PreProcessing()
{
    // Time
    ct++;
    Looptime_ =  ct/ control_frequency_; 
    // Orientation
    Euler = Utils_.Quat2EulerClamped(shared_memory_->quat);
    R_Mtrx = Utils_.Quat2RotMatrix(shared_memory_->quat);
    R_Mtrx_Trs = R_Mtrx.transpose();
    IMU_angvel = Eigen::Vector3d(shared_memory_->IMU[0], shared_memory_->IMU[1], shared_memory_->IMU[2]);
    IMU_linacc = Eigen::Vector3d(shared_memory_->IMU[3], shared_memory_->IMU[4], shared_memory_->IMU[5]);
    // Joint
    q_  =Utils_.ReMapLegOrder(shared_memory_->q);
    dq_ =Utils_.ReMapLegOrder(shared_memory_->qd);
    q_state = Utils_.SetQState(q_);
    dq_state = Utils_.SetDQState(dq_);
    // Get Kinematics & Dynamics
    GetKinematics();
    GetJacobian();
    GetDynamics();
}
    void DynamicsController::GetKinematics(){
        for (int leg = 0; leg < 4; leg++){
            Kine_.legFK(q_[3*leg+0],q_[3*leg+1],q_[3*leg+2], leg, LEG_0, LEG_1, LEG_2, ROBOT_WIDTH, ROBOT_PSIDE, R_Mtrx, foot_pos);
        }
        foot_vel.col(0) = J_.FL.J * dq_.segment<3>(0);
        foot_vel.col(1) = J_.FR.J * dq_.segment<3>(3);
        foot_vel.col(2) = J_.RL.J * dq_.segment<3>(6);
        foot_vel.col(3) = J_.RR.J * dq_.segment<3>(9);
    }
    void DynamicsController::GetJacobian(){
        J_.FL.J     = RBDL_.CalcFootJacobian(_FL_, q_state);
        J_.FL.J_Trs = J_.FL.J.transpose();
        J_.FR.J     = RBDL_.CalcFootJacobian(_FR_, q_state);
        J_.FR.J_Trs = J_.FR.J.transpose();
        J_.RL.J     = RBDL_.CalcFootJacobian(_RL_, q_state);
        J_.RL.J_Trs = J_.RL.J.transpose();
        J_.RR.J     = RBDL_.CalcFootJacobian(_RR_, q_state);
        J_.RR.J_Trs = J_.RR.J.transpose();
        
        J_.Body.Whole = RBDL_.CalcWholeJacobian(q_state);
        J_.Body.Whole_Trs.block(6,6,12,12) = RBDL_.CalcWholeJacobianTrans(J_.Body.Whole);
        J_.Body.Whole_Trs.block(0,0,6,6) = Eigen::MatrixXd::Identity(6, 6);

        J_.LEGs.block(0,0, 3, 3) = J_.FL.J;
        J_.LEGs.block(3,3, 3, 3) = J_.FR.J;
        J_.LEGs.block(6,6, 3, 3) = J_.RL.J;
        J_.LEGs.block(9,9, 3, 3) = J_.RR.J;

        RBDL_.CalcJacobianDot(LEG_0, LEG_1, LEG_2, q_(0), q_(1),    q_(2),  dq_(0), dq_(1),     dq_(2),     _FL_, J_.FL.J_dot );
        RBDL_.CalcJacobianDot(LEG_0, LEG_1, LEG_2, q_(3), q_(4),    q_(5),  dq_(3), dq_(4),     dq_(5),     _FR_, J_.FR.J_dot );
        RBDL_.CalcJacobianDot(LEG_0, LEG_1, LEG_2, q_(6), q_(7),    q_(8),  dq_(6), dq_(7),     dq_(8),     _RL_, J_.RL.J_dot );
        RBDL_.CalcJacobianDot(LEG_0, LEG_1, LEG_2, q_(9), q_(10),   q_(11), dq_(9), dq_(10),    dq_(11),    _RR_, J_.RR.J_dot );
        
        J_.Body.Whole_Dot.block(6,6,3,3)    = J_.FL.J_dot ;
        J_.Body.Whole_Dot.block(9,9,3,3)    = J_.FR.J_dot ;
        J_.Body.Whole_Dot.block(12,12,3,3)  = J_.RL.J_dot ;
        J_.Body.Whole_Dot.block(15,15,3,3)  = J_.RR.J_dot ;
    }
    void DynamicsController::GetDynamics(){
        Dyn_.M_.Body    = RBDL_.GetBodyInertiaMatrix(q_state);
        Dyn_.M_.Leg     = RBDL_.GetLegInertiaMatrix(q_state);
        Dyn_.M_.FL_.M3  = Dyn_.M_.Leg.block(0, 6, 3, 3);
        Dyn_.M_.FR_.M3  = Dyn_.M_.Leg.block(3, 9, 3, 3);
        Dyn_.M_.RL_.M3  = Dyn_.M_.Leg.block(6, 12, 3, 3);
        Dyn_.M_.RR_.M3  = Dyn_.M_.Leg.block(9, 15, 3, 3);

        Dyn_.CG_.Body    = RBDL_.GetBodyColiolisAndGravityVector(q_state, dq_state);
        Dyn_.CG_.Leg     = RBDL_.GetLegColiolisAndGravityVector(q_state, dq_state);
        Dyn_.CG_.FL_.CG3 = Dyn_.CG_.Leg.segment<3>(0);
        Dyn_.CG_.FR_.CG3 = Dyn_.CG_.Leg.segment<3>(3);
        Dyn_.CG_.RL_.CG3 = Dyn_.CG_.Leg.segment<3>(6);
        Dyn_.CG_.RR_.CG3 = Dyn_.CG_.Leg.segment<3>(9);  

        Dyn_.M_.BodyInertiaRotation = Dyn_.M_.Body.block(3, 3, 3, 3);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynamicsController::StandUpMotion(double now)
{
    // 1) 이미 완료되었으면 아무것도 하지 않음
    if (StandUp.finished) 
    {
        return;
    }

    // 2) 시작 시점 초기화
    if (!StandUp.started) {
        StandUp.started    = true;
        StandUp.startTime  = now;
        StandUp.InitPose   = q_;   // 현재 관절각 저장
        StandUp.InitVel    = dq_;  // 현재 관절속도 저장
        StandUp.RefPose    << 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6 ;
    }
    // 3) 경과 시간 및 τ 계산
    double t    = now - StandUp.startTime;
    double tau  = std::clamp(t / StandUp.duration, 0.0, 1.0);

    double s   = GC_.quintic(tau);
    double ds  = GC_.dquintic(tau) / StandUp.duration;  // dτ/dt = 1/duration

    // 4) 목표 각도 & 속도
    Eigen::VectorXd poserr  = StandUp.RefPose - StandUp.InitPose;
    Eigen::VectorXd q_ref   = StandUp.InitPose + poserr * s;
    Eigen::VectorXd dq_ref  = poserr * ds;
    Eigen::VectorXd tau_all = StandUp.Kp.cwiseProduct(q_ref  - q_) + StandUp.Kd.cwiseProduct(dq_ref - dq_);
    
    // 6) 다리별로 Dyn_.Torque에 할당
    for(int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3d tau3 = tau_all.segment<3>(3*leg);
        switch(leg) {
            case _FL_: Dyn_.Torque.FL_.Leg = tau3; break;
            case _FR_: Dyn_.Torque.FR_.Leg = tau3; break;
            case _RL_: Dyn_.Torque.RL_.Leg = tau3; break;
            case _RR_: Dyn_.Torque.RR_.Leg = tau3; break;
        }
    }

    // 7) 완료 플래그
    if (tau >= 1.0) {
        StandUp.finished = true;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynamicsController::MidProcessing()
{
    StateEstimator();
    
    if (StandUp.finished)
    {
        int gait_mode = 3;
        
        if (Looptime_ > 3.0 && Looptime_ < 10.0) {
            Walking_FLAG = true;
        }

        GaitSelector( Walking_FLAG, gait_mode);

        SwingLegController();

        StanceLegQPController();

    }
    else{
        StandUpMotion(Looptime_);
    }

}
    void DynamicsController::StateEstimator()
    {
        SetReferencePosture();
        // --- 1) 발 위치/속도 계산 ---
        Act.FL_.pos = foot_pos.col(_FL_);
        Act.FL_.vel = foot_vel.col(_FL_);

        Act.FR_.pos = foot_pos.col(_FR_);
        Act.FR_.vel = foot_vel.col(_FR_);

        Act.RL_.pos = foot_pos.col(_RL_);      
        Act.RL_.vel = foot_vel.col(_RL_);

        Act.RR_.pos = foot_pos.col(_RR_);
        Act.RR_.vel = foot_vel.col(_RR_);


        // --- 2) CoM 위치/속도 추정 ---
        Eigen::Vector3d CoMVel = Eigen::Vector3d::Zero();
        CoMVel = CalcKalmanBV();

        // X
        Act.CoM_.pos(0) = -(Act.FL_.pos(0) + Act.FR_.pos(0) + Act.RL_.pos(0) + Act.RR_.pos(0)) / 4.0;
        Act.CoM_.vel(0) = GC_.LowPassFilter(-CoMVel(0), Act.CoM_.vel(0), SensorLPFHz, dt);
        // Y
        Act.CoM_.pos(1) = -(Act.FL_.pos(1) + Act.FR_.pos(1) + Act.RL_.pos(1) + Act.RR_.pos(1)) / 4.0;
        Act.CoM_.vel(1) = GC_.LowPassFilter(-CoMVel(1), Act.CoM_.vel(1), SensorLPFHz, dt);

        // Z
        Act.CoM_.pos(2) = Kine_.calcKinematicHeight(foot_pos, 2, dt);
        Act.CoM_.vel(2) = (Act.CoM_.pos(2) - old_z) / dt;      
        old_z = Act.CoM_.pos(2);

        // Roll
        Act.CoM_.pos(3) = Euler(0);
        Act.CoM_.vel(3) = shared_memory_->gyro[0];

        // Pitch
        Act.CoM_.pos(4) = Euler(1);
        Act.CoM_.vel(4) = shared_memory_->gyro[1];

        // Yaw
        Act.CoM_.pos(5) = Euler(2);
        Act.CoM_.vel(5) = shared_memory_->gyro[2];
        
        // --- 3) world 좌표계 계산 ---
        // base linear vel in world
        Eigen::Vector3d v_base_b = Act.CoM_.vel.head<3>();      // body frame이라면
        Eigen::Vector3d v_base_w = R_Mtrx * v_base_b;           // -> world
        for (int i=0 ; i<3; i++){
            World.CoM_.vel(i) = v_base_w(i);
        }
        // angular vel
        Eigen::Vector3d w_b(shared_memory_->gyro[0],
                            shared_memory_->gyro[1],
                            shared_memory_->gyro[2]);            // gyro는 보통 body frame
        Eigen::Vector3d w_w = R_Mtrx * w_b;                      // -> world
        
        auto footAbsVelWorld = [&](const State::Foot& fb)->Eigen::Vector3d
        {
            const Eigen::Vector3d r_b = fb.pos;                  // p_f^b
            const Eigen::Vector3d r_w = R_Mtrx * r_b;            // R p_f^b
            const Eigen::Vector3d v_rel_w = R_Mtrx * fb.vel;     // R v_rel^b

            // v_f^w = v_base^w + w^w x r^w + v_rel^w
            return v_base_w + w_w.cross(r_w) + v_rel_w;
        };

        // 월드 절대 발속도 (원하면 World.FL_.vel에 저장)
        World.FL_.vel = footAbsVelWorld(Act.FL_);
        World.FR_.vel = footAbsVelWorld(Act.FR_);
        World.RL_.vel = footAbsVelWorld(Act.RL_);
        World.RR_.vel = footAbsVelWorld(Act.RR_);

        // - - - 경사 추정      - - - //
        // EstimateSlope();

        // - - - 미끄러짐 추정  - - - //
        slip_Prob = CalcSlipProb(Pattern_.SW_FLAG, foot_vel);

        // - - - 착지 힘 추정   - - - //
        // contact estimation
        // - - - 외란 추정      - - - //
        Eigen::VectorXd CoMdistForce = Eigen::VectorXd::Zero(12);
        CoMdistForce = CalcKalmanDoB();
        Disturbance = CoMdistForce.segment<6>(6); // 외란 힘

    }
        // void DynamicsController::EstimateSlope()
        // {
        //     std::vector<SlopeEstimator::InputFoot> stance;
        //     stance.reserve(4);
        //     auto add_if_stance = [&](int leg, const Eigen::Vector3d& p_w, const Eigen::Vector3d& v_w) {
        //         if (Pattern_.SW_FLAG[leg] != 0) return; // 0: stance, 1: swing(ON)
        //         SlopeEstimator::InputFoot f;
        //         f.id = leg;
        //         // foot_pos / foot_vel이 이미 월드(or gravity-aligned world)라고 가정
        //         f.p_w = p_w;
        //         // 가중치(권장): stance라도 미끄러지거나 발이 흔들리면 평면이 깨짐
        //         // 속도가 클수록 가중치 낮추기 (간단 버전)
        //         const double v = v_w.norm();
        //         const double sigma = 0.2; // [m/s] 튜닝
        //         f.weight = std::exp(-(v*v)/(2.0*sigma*sigma));
        //         stance.push_back(f);
        //     };
        //     add_if_stance(_FL_, Act.FL_.pos, Act.FL_.vel);
        //     add_if_stance(_FR_, Act.FR_.pos, Act.FR_.vel);
        //     add_if_stance(_RL_, Act.RL_.pos, Act.RL_.vel);
        //     add_if_stance(_RR_, Act.RR_.pos, Act.RR_.vel);
        //     slope_out_ = slope_est_.update(stance);
        //     // 결과 저장
        //     Slope_.ground_normal_w_ = slope_out_.normal_w;
        //     Slope_.ground_roll_  = slope_out_.roll;   // rad
        //     Slope_.ground_pitch_ = slope_out_.pitch;  // rad
        // }
        Eigen::Vector4d DynamicsController::CalcSlipProb( bool SW[4], Eigen::MatrixXd& FV)
        {
            std::array<bool,4>              SW_Flag_SD; 
            std::array<Eigen::Vector3d,4>   footVel_SD;
            Eigen::Vector4d SlipProb = Eigen::Vector4d::Zero();
            for(int leg=0; leg<4; ++leg){
                SW_Flag_SD[leg] = !SW[leg];
                footVel_SD[leg] = FV.col(leg);
                if (SW_Flag_SD[leg]) {
                    SD_.updateLeg(leg, SW_Flag_SD[leg], footVel_SD[leg]);
                    if (SD_.isSlip(leg)) {
                        std::cout << "[Warning] Leg " << leg << " slip! belief=" << SD_.getBelief(leg) << std::endl;
                    }
                }
                SlipProb[leg] = SD_.getBelief(leg); 
            }
            return SlipProb;
        } 
        Eigen::Vector3d DynamicsController::CalcKalmanBV()
        {
            Eigen::Matrix<double,18,18> A = Eigen::Matrix<double,18,18>::Identity();
            A.block<3,3>(0, 3) = I3 * dt;
            BodyVelocity.setA(A);

            Eigen::Matrix<double,18,3> B =Eigen::Matrix<double,18,3>::Zero();
            B << I3*(dt*dt/2), I3*dt, Z3, Z3, Z3, Z3;                 
            BodyVelocity.setB(B);

            Eigen::Matrix<double,24,18> C = Eigen::Matrix<double,24,18>::Zero();
            for (int i = 0; i < 4; ++i) {
                C.block<3,3>(3*i, 3*0) = -I3;
                C.block<3,3>(3*i, 3*(2 + i)) = I3;
                C.block<3,3>(3*(4 + i), 3*1) = -I3;
            }
            BodyVelocity.setC(C);
            Eigen::Matrix<double,24,1> z = Eigen::Matrix<double,24,1>::Zero();
            for (int i = 0; i < 4; ++i) {
                z.segment<3>(3*i) = -foot_pos.col(i);
                z.segment<3>(12 + 3*i) = -foot_vel.col(i);
            }
            Eigen::Vector3d u = Eigen::Vector3d::Zero();
            if (Walking_FLAG == true) {
                u(0) = IMU_linacc[0]; 
                u(1) = IMU_linacc[1];
                u(2) = IMU_linacc[2] - Gravity; // Z축 가속도
            }
            else {
                u(0) = 0; 
                u(1) = 0;
                u(2) = shared_memory_->IMU[5] - Gravity; 
            }

            Eigen::Matrix<double,24,24> R = Eigen::Matrix<double,24,24>::Zero();
            double o_noise1=0.000002;	        double o_noise2=0.02;
            std::array<double,4> o_scale = {0.49, 0.49, 0.49, 0.49};
            if (Walking_FLAG) {
                for (int i = 0; i < 4; ++i) {
                    // Pattern_.SW_FLAG[i] == 1 → 80 + dt, 0 → dt
                    o_scale[i] = -1 * ( -80.0 * Pattern_.SW_FLAG[i] - dt );
                }
            }
            for (int i = 0; i < 4; ++i) {
                R.block<3,3>(3*i,     3*i    ) = I3 * (o_noise1 * o_scale[i]);
                R.block<3,3>(3*(4+i), 3*(4+i)) = I3 * (o_noise2 * o_scale[i]);
            }

            BodyVelocity.setR(R);   
            BodyVelocity.predict(u);
            BodyVelocity.update(z, u);
            Eigen::VectorXd x_est = BodyVelocity.getState();
            return x_est.segment(3, 3);
        }
        Eigen::VectorXd DynamicsController::CalcKalmanDoB()
        {
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(SRB_model_.stateDim, SRB_model_.stateDim);
            A.block<3, 3>(0, 0) = I3;
            A.block<3, 3>(3, 3) = I3;
            A.block<3, 3>(0, 6) = (dt / ROBOT_M) * I3;
            A.block<3, 3>(3, 9) = dt * Dyn_.M_.BodyInertiaRotation.inverse();  
            A.block<3, 3>(6, 6) = I3;
            A.block<3, 3>(9, 9) = I3;
            DoB.setA(A);
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero(SRB_model_.stateDim, SRB_model_.inputDim);
            Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(12, 12); 
            for (int i = 0; i < 4; ++i)
            {
                double trust = (Pattern_.SW_FLAG[i] == 0) ? 1.0 : 0.01; 
                B.block<3, 3>(0, i * 3) = (dt / ROBOT_M) * I3;
                Eigen::Vector3d r_i = foot_pos.col(i);
                Eigen::Matrix3d skew_r_i;
                skew_r_i <<      0, -r_i(2),  r_i(1),
                            r_i(2),       0, -r_i(0),
                           -r_i(1),  r_i(0),       0;
                // B.block<3, 3>(3, i * 3) = -dt * skew_r_i;
                B.block<3, 3>(9, i * 3) = -dt * skew_r_i;
                W_b.block<3,3>(i*3, i*3) *= trust;
            }
            DoB.setB(B* W_b);
            // 3) C, D  (측정 y = [a_imu; w_imu])
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(SRB_model_.outputDim, SRB_model_.stateDim); // 6x12
            Eigen::MatrixXd D = Eigen::MatrixXd::Zero(SRB_model_.outputDim, SRB_model_.inputDim); // 6x12
            // 가속도 3행: (1/m) * Fd
            C.block<3,3>(0, 6) = (1.0 / ROBOT_M) * I3;
            // 각속도 3행: w
            C.block<3,3>(3, 3) = I3;
            // D: 가속도 3행에 (1/m)*[I I I I], 각속도 3행은 0
            for (int i = 0; i < 4; ++i) {
                D.block<3,3>(0, i*3) = (1.0 / ROBOT_M) * I3;
            }
            DoB.setC(C);
            DoB.setD(D);
            Eigen::VectorXd u = Eigen::VectorXd::Zero(SRB_model_.inputDim);
            u.segment<3>(0) = Dyn_.Force.FL_.QPLambda;
            u.segment<3>(3) = Dyn_.Force.FR_.QPLambda;
            u.segment<3>(6) = Dyn_.Force.RL_.QPLambda;
            u.segment<3>(9) = Dyn_.Force.RR_.QPLambda;
            Eigen::VectorXd u_scaled = u;
            for (int i = 0; i < 4; ++i) {
                double trust = (Pattern_.SW_FLAG[i] == 0) ? 1.0 : 0.01;
                u_scaled.segment<3>(i*3) *= trust;
            }
            Eigen::VectorXd y = Eigen::VectorXd::Zero(SRB_model_.outputDim);
            y.segment<3>(0) = IMU_linacc;      
            y.segment<3>(3) = IMU_angvel;   
            DoB.predict(u_scaled);
            DoB.update(y,u_scaled);
            return DoB.getState();
        }
    
        void DynamicsController::SetReferencePosture()
        {
            Ref.CoM_.pos(0) = 0;
            Ref.CoM_.pos(1) = 0;
            Ref.CoM_.pos(2) = ROBOT_STAND_HEIGHT; 

            Ref.CoM_.pos(3) = 0; // Roll
            Ref.CoM_.pos(4) = 0; // Pitch
            Ref.CoM_.pos(5) = 0; // Yaw
    
            Ref.CoM_.vel(0) = shared_memory_->lin_vel_target[0] *0.8;
            Ref.CoM_.vel(1) = shared_memory_->lin_vel_target[1] *0.8;
            Ref.CoM_.vel(2) = 0;

            Ref.CoM_.vel(3) = 0; // Roll
            Ref.CoM_.vel(4) = 0; // Pitch
            Ref.CoM_.vel(5) =  shared_memory_->ang_vel_target[2] *1; // Yaw
        }
    void DynamicsController::GaitSelector( bool WALKING_FLAG_, int Gait)
    {
        // --- 1) 시간 업데이트 ---
        Pattern_.ct++;
        Pattern_.time = Pattern_.ct / Pattern_.hz;

        // --- 2) 수동 변경 요청 감지 (즉시 new_cycle 은 트리거하지 않음) ---
        if (WALKING_FLAG_  != Pattern_.prev_walking_flag || Gait != Pattern_.prev_gait_mode)
        {
            Pattern_.req_walking_flag = WALKING_FLAG_;
            Pattern_.req_gait_mode    = Gait;
        }

        // --- 3) 새 사이클 시작 판별: 첫 사이클 or 사이클 만료(=timeout) 일 때만 ---
        bool timeout   = (Pattern_.time >= Pattern_.cycle_start_time + Pattern_.cycle_length);
        bool new_cycle = Pattern_.first_cycle || timeout;

        if (new_cycle) {
            // --- 3.1) 사이클 경계 진입 시 요청이 있으면 prev 에 반영 ---
            if (Pattern_.first_cycle) {
            } 
            else if (Pattern_.req_gait_mode != Pattern_.prev_gait_mode || Pattern_.req_walking_flag != Pattern_.prev_walking_flag)
            {
                Pattern_.prev_gait_mode    = Pattern_.req_gait_mode;
                Pattern_.prev_walking_flag = Pattern_.req_walking_flag;
            }

            // --- 3.2) 사이클 초기화 ---
            Pattern_.cycle_start_time = Pattern_.time;
            Pattern_.first_cycle      = false;

            bool walking = Pattern_.prev_walking_flag;
            int  gait    = Pattern_.prev_gait_mode;

            if (!walking) {
                // 보행 OFF: 다음 사이클부터 완전 스탠스
                Pattern_.cycle_length = 0.0;
                Pattern_.time_sw      = 0.0;
                Pattern_.time_st      = 0.0;
            }
            else {
                // 페이즈 설정 (TROT=1, WALK=2)
                switch (gait) {
                    case 1:
                        Pattern_.cycle_length = Trot_Cycle_Length;
                        Pattern_.time_sw      = (Pattern_.cycle_length / 2.0) * 0.9;
                        break;
                    case 2:
                        Pattern_.cycle_length = Walk_Cycle_Length;
                        Pattern_.time_sw      = (Pattern_.cycle_length / 4.0) * 0.9;
                        break;
                    case 3:
                        Pattern_.cycle_length = FastTrot_Cycle_Length;
                        Pattern_.time_sw      = (Pattern_.cycle_length / 2.0) * 0.9;
                        break;
                    case 4:
                        Pattern_.cycle_length = FlyingTrot_Cycle_Length;
                        Pattern_.time_sw      = (Pattern_.cycle_length / 2.0) * 1.3;
                        break;
                    default:
                        Pattern_.cycle_length = 0.0;
                        Pattern_.time_sw      = 0.0;
                }
                Pattern_.time_st = Pattern_.cycle_length - Pattern_.time_sw;
            }

            // --- 3.3) Swing 플래그 & 타이밍 재설정 ---
            for (int leg = 0; leg < 4; ++leg) {
                Pattern_.SW_FLAG[leg] = DOWN;
            }
            if (walking) {
                const double* pattern = (gait == 2)
                    ? Pattern_.walk_pattern_ratio
                    : Pattern_.trot_pattern_ratio;
                for (int leg = 0; leg < 4; ++leg) {
                    double up = pattern[leg] * Pattern_.cycle_length;
                    Pattern_.Sw_timing[UP][leg]   = Pattern_.cycle_start_time + up;
                    Pattern_.Sw_timing[DOWN][leg] = Pattern_.Sw_timing[UP][leg] + Pattern_.time_sw;
                }
            }
        }

        // --- 4) Swing/Stand 플래그 업데이트 (언제나 기존 페이즈 유지) ---
        for (int leg = 0; leg < 4; ++leg) {
            if (Pattern_.prev_walking_flag && Pattern_.time >= Pattern_.Sw_timing[UP][leg] && Pattern_.time <  Pattern_.Sw_timing[UP][leg] + Pattern_.time_sw && Pattern_.SW_FLAG[leg] == DOWN)
            {
                Pattern_.SW_FLAG[leg] = ON;
            }
            else if (Pattern_.SW_FLAG[leg] == ON && Pattern_.time >= Pattern_.Sw_timing[DOWN][leg])
            {
                Pattern_.SW_FLAG[leg] = DOWN;
                // 이 순간의 실제 발 위치를 저장
                Pattern_.stanceXtraj[leg] = foot_pos(_X_, leg);
                Pattern_.stanceYtraj[leg] = foot_pos(_Y_, leg);
                Pattern_.stanceZtraj[leg] = -ROBOT_STAND_HEIGHT; // Z축은 항상 로봇 높이로 고정
            }
        }

        // --- 5) Swing Phase(0→1) 업데이트 ---
        for (int leg = 0; leg < 4; ++leg) {
            if (Pattern_.SW_FLAG[leg] == ON) {
                // 스윙 시작 시간 기준으로 정규화
                double t0    = Pattern_.Sw_timing[UP][leg];
                double phase = (Pattern_.time - t0) / Pattern_.time_sw;
                // 범위 클램프
                if (phase < 0.0)   phase = 0.0;
                if (phase > 1.0)   phase = 1.0;
                Pattern_.SwingPhase[leg] = phase;
            }
            else {
                Pattern_.SwingPhase[leg] = 0.0;
            }
        }
    }
    void DynamicsController::SwingLegController()
    {
        
        RaibertOffset = CalcRaibertHeuristic(Ref.CoM_.vel.segment(0,3), Act.CoM_.vel.segment(0,3), shared_memory_->ang_vel_target[2]); 
        CapturePointOffset = CalcCapturePoint(-Act.CoM_.vel(1), 0.5); 

        Eigen::Vector4d baseX, baseY;
        for(int leg=0; leg<4; ++leg) {
            baseX[leg] = ((leg==_FL_ || leg==_FR_) ? 1.0 : -1.0) * (ROBOT_WIDTH  / 2.0);
            baseY[leg] = ((leg==_FL_ || leg==_RL_) ? 1.0 : -1.0) * (ROBOT_PSIDE / 2.0);
        }

        Eigen::Vector4d targetX = baseX + RaibertOffset.row(0).transpose();
        Eigen::Vector4d targetY = baseY + RaibertOffset.row(1).transpose() + CapturePointOffset; 
        ref_foot_pos = CalcQuinticSwingTrajectory(targetX, targetY, Pattern_.SwingPhase);

        Ref.FL_.pos = ref_foot_pos.col(_FL_);
        Ref.FR_.pos = ref_foot_pos.col(_FR_);
        Ref.RL_.pos = ref_foot_pos.col(_RL_);
        Ref.RR_.pos = ref_foot_pos.col(_RR_);

        // CalcSwingForceControl(false);    // false: CTC 모드가 아닌 Task space 모드로 스윙 포스 계산
        CalcSwingForceControl(true);        // true: CTC 모드로 스윙 포스 계산
    }
        Eigen::Matrix<double,2,4>  DynamicsController::CalcRaibertHeuristic(const Eigen::Vector3d& RefVel, const Eigen::Vector3d& ActVel, double omega_cmd) 
        {
            Eigen::Matrix<double,2,4> RaibertOffset;
            double omega0 = std::sqrt(ROBOT_STAND_HEIGHT / Gravity);
            double halfT  = Pattern_.time_st * 0.5;

            // 기본 Raibert heuristic
            double dx_lin = RefVel(0) * halfT + omega0 * (ActVel(0) - RefVel(0));
            double dy_lin = RefVel(1) * halfT + omega0 * (ActVel(1) - RefVel(1));

            // 회전 항 (cross-term)
            double alpha   = 0.5 * omega0;      // = ½√(h/g)
            double dx_rot  =  alpha * ActVel(1) * omega_cmd;   // v_y * ω
            double dy_rot  = -alpha * ActVel(0) * omega_cmd;   // -v_x * ω

            double dx = dx_lin + dx_rot;
            double dy = dy_lin + dy_rot;

            // 모든 다리에 동일하게 적용
            RaibertOffset.row(0).setConstant(dx);  // X-offset
            RaibertOffset.row(1).setConstant(dy);  // Y-offset
            return RaibertOffset;
        }
        Eigen::Vector4d DynamicsController::CalcCapturePoint(double Act_Com_y_vel, double CP_ratio)
        {
            double LPF_y_vel   = Act_Com_y_vel;
            double cp_scale = CP_ratio * std::sqrt(ROBOT_STAND_HEIGHT / Gravity);
            Eigen::Vector4d swingMask;
            for (int i = 0; i < 4; ++i) {swingMask(i) = (Pattern_.SW_FLAG[i] == 1 ? 1.0 : 0.0);}
            Eigen::Vector4d now_capturepoint = Eigen::Vector4d::Constant(LPF_y_vel);
            CP_Dist_Upper = CP_Dist_Upper.cwiseMax(now_capturepoint).cwiseProduct(swingMask);
            CP_Dist_Lower = CP_Dist_Lower.cwiseMin(now_capturepoint).cwiseProduct(swingMask);
            Eigen::Vector4d absU = CP_Dist_Upper.cwiseAbs();
            Eigen::Vector4d absL = CP_Dist_Lower.cwiseAbs();
            Eigen::Vector4d chosen = (absU.array() > absL.array()).select(CP_Dist_Upper, CP_Dist_Lower);
            Eigen::Vector4d CapturePoint = chosen * cp_scale;
            return CapturePoint;
        }
        Eigen::Matrix<double,3,4>  DynamicsController::CalcQuinticSwingTrajectory(Eigen::Vector4d& RefX,Eigen::Vector4d& RefY, double SwingPhase[4])
        {
            Eigen::Matrix<double,3,4>  swing_trajectory(3,4);

            for (int leg = 0; leg < 4; ++leg) {
                // 스탠스→스윙 전환 시 시작 위치 저장
                if (Pattern_.old_SW_FLAG[leg] == 0 && Pattern_.SW_FLAG[leg] == 1) {
                    SwingStartX[leg] = foot_pos(_X_, leg);
                    SwingStartY[leg] = foot_pos(_Y_, leg);
                    SwingStartZ[leg] = foot_pos(_Z_, leg);
                }

                if (Pattern_.SW_FLAG[leg] == 1) {
                    // Normalize phase to [0,1]
                    double tau = std::clamp(SwingPhase[leg], 0.0, 1.0);

                    // Interpolate X and Y between start (SwingStart*) and target (Ref*)
                    double x = SwingStartX[leg] + (RefX[leg] - SwingStartX[leg]) * GC_.quintic(tau);
                    double y = SwingStartY[leg] + (RefY[leg] - SwingStartY[leg]) * GC_.quintic(tau);

                    double z;
                    if (tau < 0.5) {
                        // rising phase
                        double phase = tau * 2.0;
                        z = SwingStartZ[leg] + ROBOT_SWING_HEIGHT * GC_.quintic(phase);
                    } else {
                        // falling phase
                        double phase = (tau - 0.5) * 2.0;
                        z = SwingStartZ[leg] + ROBOT_SWING_HEIGHT * (1.0 - GC_.quintic(phase));
                    }
                    swing_trajectory(0, leg) = x;
                    swing_trajectory(1, leg) = y;
                    swing_trajectory(2, leg) = z;

                    Pattern_.old_SW_FLAG[leg] = Pattern_.SW_FLAG[leg];

                }
               
                else if (Pattern_.SW_FLAG[leg] == 0) {
                    swing_trajectory(0, leg) = Pattern_.stanceXtraj[leg];
                    swing_trajectory(1, leg) = Pattern_.stanceYtraj[leg];
                    swing_trajectory(2, leg) = Pattern_.stanceZtraj[leg];
                    Pattern_.old_SW_FLAG[leg] = 0;
                    continue;
                }
            }
            return swing_trajectory;
        }
        void DynamicsController::CalcSwingForceControl(bool mode)
        {
            if( mode ==true ){
                Dyn_.Torque.FL_.SwingTorque = CalcTaskSpaceCTC(Ref.FL_.pos, Ref.FL_.vel, Ref.FL_.acc, Act.FL_.pos, Act.FL_.vel, dq_.segment<3>(0), J_.FL.J,J_.FL.J_dot, Dyn_.M_.FL_.M3, Dyn_.CG_.FL_.CG3);
                Dyn_.Torque.FR_.SwingTorque = CalcTaskSpaceCTC(Ref.FR_.pos, Ref.FR_.vel, Ref.FR_.acc, Act.FR_.pos, Act.FR_.vel, dq_.segment<3>(3), J_.FR.J,J_.FR.J_dot, Dyn_.M_.FR_.M3, Dyn_.CG_.FR_.CG3);
                Dyn_.Torque.RL_.SwingTorque = CalcTaskSpaceCTC(Ref.RL_.pos, Ref.RL_.vel, Ref.RL_.acc, Act.RL_.pos, Act.RL_.vel, dq_.segment<3>(6), J_.RL.J,J_.RL.J_dot, Dyn_.M_.RL_.M3, Dyn_.CG_.RL_.CG3);
                Dyn_.Torque.RR_.SwingTorque = CalcTaskSpaceCTC(Ref.RR_.pos, Ref.RR_.vel, Ref.RR_.acc, Act.RR_.pos, Act.RR_.vel, dq_.segment<3>(9), J_.RR.J,J_.RR.J_dot, Dyn_.M_.RR_.M3, Dyn_.CG_.RR_.CG3);
            }
            else{
                Dyn_.Force.FL_.EEForce = CalcTaskSpaceForce(Ref.FL_.pos, Act.FL_.pos, Ref.FL_.vel, Act.FL_.vel);
                Dyn_.Torque.FL_.SwingTorque = J_.FL.J_Trs * R_Mtrx_Trs * Dyn_.Force.FL_.EEForce;
                Dyn_.Force.FR_.EEForce = CalcTaskSpaceForce(Ref.FR_.pos, Act.FR_.pos, Ref.FR_.vel, Act.FR_.vel);
                Dyn_.Torque.FR_.SwingTorque = J_.FR.J_Trs * R_Mtrx_Trs * Dyn_.Force.FR_.EEForce;
                Dyn_.Force.RL_.EEForce = CalcTaskSpaceForce(Ref.RL_.pos, Act.RL_.pos, Ref.RL_.vel, Act.RL_.vel);
                Dyn_.Torque.RL_.SwingTorque = J_.RL.J_Trs * R_Mtrx_Trs * Dyn_.Force.RL_.EEForce;
                Dyn_.Force.RR_.EEForce = CalcTaskSpaceForce(Ref.RR_.pos, Act.RR_.pos, Ref.RR_.vel, Act.RR_.vel);
                Dyn_.Torque.RR_.SwingTorque = J_.RR.J_Trs * R_Mtrx_Trs * Dyn_.Force.RR_.EEForce;
            }
        }
            Eigen::Vector3d DynamicsController::CalcTaskSpaceForce( Eigen::Vector3d& Ref_Pos,Eigen::Vector3d& Act_Pos, Eigen::Vector3d& Ref_Vel,Eigen::Vector3d& Act_Vel)
            {
                Eigen::Vector3d task_space_force = Eigen::Vector3d::Zero();
                Eigen::Vector3d pos_err = Ref_Pos - Act_Pos;
                Eigen::Vector3d vel_err = Ref_Vel - Act_Vel;
                const double P_gain = robot_config_.swing_taskspace.kp;
                const double D_gain = robot_config_.swing_taskspace.kd; 
                task_space_force = P_gain * pos_err + D_gain * vel_err;
                return task_space_force;
            }
            Eigen::Vector3d DynamicsController::CalcTaskSpaceCTC(   const Eigen::Vector3d& x_d, const Eigen::Vector3d& dx_d, const Eigen::Vector3d& ddx_d, 
                                                                    const Eigen::Vector3d& x, const Eigen::Vector3d& dx,    const Eigen::Vector3d& q_dot_,
                                                                    const Eigen::Matrix3d& J, const Eigen::Matrix3d& J_dot_, 
                                                                    const Eigen::Matrix3d& Mq, const Eigen::Vector3d& CG  )
            {
                const double kp_scalar = robot_config_.swing_ctc.kp;
                const double kd_scalar = robot_config_.swing_ctc.kd;

                const Eigen::Matrix3d Kp = kp_scalar * Eigen::Matrix3d::Identity();
                const Eigen::Matrix3d Kd = kd_scalar * Eigen::Matrix3d::Identity();
            
                const Eigen::Vector3d e_x  = x_d - x;
                const Eigen::Vector3d e_dx = dx_d - dx;

                const Eigen::Vector3d ddx_des = ddx_d + Kp * e_x + Kd * e_dx;
            
                Eigen::Vector3d rhs = ddx_des - J_dot_ * q_dot_;
                Eigen::Vector3d ddq_des = J.fullPivLu().solve(rhs);
            
                return Mq * ddq_des + CG;
            }
    void DynamicsController::StanceLegQPController()
    {
        applyGainsToQP(Walking_FLAG);

        // Equality 
        Eigen::Matrix<double,6,30>  Aeq   = PS_->CalcEqualityConstraint_A(foot_pos, Dyn_.M_.Body, Pattern_.SW_FLAG);
        Eigen::Matrix<double,6,1 >  Beq   = PS_->CalcEqualityConstraint_B(Dyn_.CG_.Body);
        // Inequality
        Eigen::Matrix<double,24,30> Aineq = PS_->CalcInequalityConstraint_A(Mu);
        Eigen::Matrix<double,24,1>  Bineq = PS_->CalcInequalityConstraint_B(-10.0, 300.0, Pattern_.SW_FLAG);
        // Cost function  
        Eigen::Matrix<double,30,30> CostA = PS_->CalcCostFuntion_A(J_.Body.Whole_Trs); 

        Eigen::Matrix<double,18,1 > Xddot = PS_->GetXddotVector(QP_.CoM_.KP, Ref.CoM_.pos.segment(0,3), Act.CoM_.pos.segment(0,3), 
                                                                QP_.CoM_.KD, Ref.CoM_.vel.segment(0,3), Act.CoM_.vel.segment(0,3),
                                                                QP_.RPY_.KP, Ref.CoM_.pos.segment(3,3), Act.CoM_.pos.segment(3,3),
                                                                QP_.RPY_.KD, Ref.CoM_.vel.segment(3,3), Act.CoM_.vel.segment(3,3) );

        Eigen::Matrix<double,30,1 > CostB = PS_->CalcCostFuntion_B(Xddot, J_.Body.Whole_Dot, dq_state, J_.Body.Whole_Trs);
        Eigen::Matrix<double,30,1 > solution = PS_->WBC_QP(CostA, CostB, Aeq, Beq, Aineq, Bineq, J_.Body.Whole_Trs, Dyn_.M_.Leg);
        Eigen::Matrix<double,12,30 >WBC_Final_Torque_Mtrx = Eigen::Matrix<double,12,30>::Zero();
        WBC_Final_Torque_Mtrx.block(0, 0,  12, 18) = Dyn_.M_.Leg.block(0, 0, 12, 18);
        WBC_Final_Torque_Mtrx.block(0, 18, 12, 12) = J_.LEGs.transpose();
        WBC_Final_Torque= WBC_Final_Torque_Mtrx * solution; 

        // 스탠스 다리에만 토크 할당 
        if (Pattern_.SW_FLAG[_FL_] == 0) {
            Dyn_.Torque.FL_.Leg = WBC_Final_Torque.segment<3>(0);
            Dyn_.Force.FL_.QPLambda = solution.segment(18,3);
        } else {
            Dyn_.Torque.FL_.Leg = Dyn_.Torque.FL_.SwingTorque;
        }
        if (Pattern_.SW_FLAG[_FR_] == 0) {
            Dyn_.Torque.FR_.Leg = WBC_Final_Torque.segment<3>(3);
            Dyn_.Force.FR_.QPLambda = solution.segment(21,3);
        } else {
            Dyn_.Torque.FR_.Leg = Dyn_.Torque.FR_.SwingTorque;
        }
        if (Pattern_.SW_FLAG[_RL_] == 0) {
            Dyn_.Torque.RL_.Leg = WBC_Final_Torque.segment<3>(6);
            Dyn_.Force.RL_.QPLambda = solution.segment(24,3);
        } else {
            Dyn_.Torque.RL_.Leg = Dyn_.Torque.RL_.SwingTorque;
        }
        if (Pattern_.SW_FLAG[_RR_] == 0) {
            Dyn_.Torque.RR_.Leg = WBC_Final_Torque.segment<3>(9);
            Dyn_.Force.RR_.QPLambda = solution.segment(27,3);
        } else {
            Dyn_.Torque.RR_.Leg = Dyn_.Torque.RR_.SwingTorque;
        }

    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynamicsController::PostProcessing()
{
    Eigen::Vector3d JointDamping;
    JointDamping << 0.1, 0.1, 0.1;
    applyVirtualDriveDamping(JointDamping);
    for (int i = 0; i < 3; ++i) {
        // RBQ
        // shared_memory_->Dyn_torque[i]   = Dyn_.Torque.RR_.Leg[i];
        // shared_memory_->Dyn_torque[i+3] = Dyn_.Torque.RL_.Leg[i];
        // shared_memory_->Dyn_torque[i+6] = Dyn_.Torque.FR_.Leg[i];
        // shared_memory_->Dyn_torque[i+9] = Dyn_.Torque.FL_.Leg[i];

        // Unitree
        shared_memory_->Dyn_torque[i]   = Dyn_.Torque.FR_.Leg[i];
        shared_memory_->Dyn_torque[i+3] = Dyn_.Torque.FL_.Leg[i];
        shared_memory_->Dyn_torque[i+6] = Dyn_.Torque.RR_.Leg[i];
        shared_memory_->Dyn_torque[i+9] = Dyn_.Torque.RL_.Leg[i];
        
        // Basic
        // shared_memory_->Dyn_torque[i]   = Dyn_.Torque.FL_.Leg[i];
        // shared_memory_->Dyn_torque[i+3] = Dyn_.Torque.FR_.Leg[i];
        // shared_memory_->Dyn_torque[i+6] = Dyn_.Torque.RL_.Leg[i];
        // shared_memory_->Dyn_torque[i+9] = Dyn_.Torque.RR_.Leg[i];
        
    }
    TCPDataStreamer();
}
    void DynamicsController::TCPDataStreamer()
    {
        std::ostringstream oss;
        oss << "footvelFLx=" << Act.FL_.vel(0) << 
            ",footvelFLy=" << Act.FL_.vel(1) << 
            ",footvelFLz=" << Act.FL_.vel(2) <<
            ",footvelFRx=" << Act.FR_.vel(0) << 
            ",footvelFRy=" << Act.FR_.vel(1) << 
            ",footvelFRz=" << Act.FR_.vel(2) <<
            ",footvelRLx=" << Act.RL_.vel(0) << 
            ",footvelRLy=" << Act.RL_.vel(1) << 
            ",footvelRLz=" << Act.RL_.vel(2) <<
            ",footvelRRx=" << Act.RR_.vel(0) << 
            ",footvelRRy=" << Act.RR_.vel(1) << 
            ",footvelRRz=" << Act.RR_.vel(2) <<
            "\n";
        tcp_server_.sendData(oss.str());
    }

DynamicsController::RobotConfig DynamicsController::loadConfig(const std::string& filename) 
{
    YAML::Node cfg = YAML::LoadFile(filename);
    RobotConfig rc;
    rc.name = cfg["robot"]["name"].as<std::string>();

    for (auto mode_it : cfg["WBC"]) {
        std::string mode_name = mode_it.first.as<std::string>(); 
        ModeGains mg;
        for (auto axis_it : mode_it.second) {
            std::string axis = axis_it.first.as<std::string>();
            Gain g;
            g.kp = axis_it.second["kp"].as<double>();
            g.kd = axis_it.second["kd"].as<double>();
            mg[axis] = g;
        }
        rc.wbc[mode_name] = mg;
    }

    // 추가 01
    if (cfg["SwingLeg"]) {
        // TaskSpace
        if (cfg["SwingLeg"]["TaskSpace"]) {
            rc.swing_taskspace.kp = cfg["SwingLeg"]["TaskSpace"]["kp"].as<double>();
            rc.swing_taskspace.kd = cfg["SwingLeg"]["TaskSpace"]["kd"].as<double>();
        }

        // ComputedTorqueControl
        if (cfg["SwingLeg"]["ComputedTorqueControl"]) {
            rc.swing_ctc.kp = cfg["SwingLeg"]["ComputedTorqueControl"]["kp"].as<double>();
            rc.swing_ctc.kd = cfg["SwingLeg"]["ComputedTorqueControl"]["kd"].as<double>();
        }
    }
    // 추가 01

    return rc;
}
void DynamicsController::applyGainsToQP(bool walking_flag) 
{
    // 모드 결정
    const std::string mode = walking_flag ? "Walking" : "Standing";
    const ModeGains& mg = robot_config_.wbc.at(mode);

    // CoM (X, Y, Z)
    QP_.CoM_.KP << mg.at("X").kp, mg.at("Y").kp, mg.at("Z").kp;
    QP_.CoM_.KD << mg.at("X").kd, mg.at("Y").kd, mg.at("Z").kd;

    // RPY (roll, pitch, yaw)
    QP_.RPY_.KP << mg.at("roll").kp,  mg.at("pitch").kp,  mg.at("yaw").kp;
    QP_.RPY_.KD << mg.at("roll").kd,  mg.at("pitch").kd,  mg.at("yaw").kd;
}
void DynamicsController::applyVirtualDriveDamping(Eigen::Vector3d& JointDamping)
{
    Dyn_.Torque.FL_.Leg(0) -= JointDamping[0]*dq_(0);
    Dyn_.Torque.FL_.Leg(1) -= JointDamping[1]*dq_(1);
    Dyn_.Torque.FL_.Leg(2) -= JointDamping[2]*dq_(2);

    Dyn_.Torque.FR_.Leg(0) -= JointDamping[0]*dq_(3);
    Dyn_.Torque.FR_.Leg(1) -= JointDamping[1]*dq_(4);
    Dyn_.Torque.FR_.Leg(2) -= JointDamping[2]*dq_(5);

    Dyn_.Torque.RL_.Leg(0) -= JointDamping[0]*dq_(6);
    Dyn_.Torque.RL_.Leg(1) -= JointDamping[1]*dq_(7);
    Dyn_.Torque.RL_.Leg(2) -= JointDamping[2]*dq_(8);
    
    Dyn_.Torque.RR_.Leg(0) -= JointDamping[0]*dq_(9);
    Dyn_.Torque.RR_.Leg(1) -= JointDamping[1]*dq_(10);
    Dyn_.Torque.RR_.Leg(2) -= JointDamping[2]*dq_(11);

}
    