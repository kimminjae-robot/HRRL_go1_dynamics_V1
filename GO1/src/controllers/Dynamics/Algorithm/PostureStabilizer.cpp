#include "PostureStabilizer.h"

PostureStablizer::PostureStablizer() {
}

PostureStablizer::~PostureStablizer() {
}


////////// WBC QP //////////
Eigen::VectorXd PostureStablizer::WBC_QP(   Eigen::MatrixXd CostFuntion_A_,         Eigen::VectorXd CostFuntion_B_, 
                                            Eigen::MatrixXd EqualityConstraint_A_,  Eigen::VectorXd EqualityConstraint_B_, 
                                            Eigen::MatrixXd InequalityConstraint_A_,Eigen::VectorXd InequalityConstraint_B_,
                                            Eigen::MatrixXd J_legs_Trs_,            Eigen::MatrixXd Inertia12_){
    
    Eigen::MatrixXd Torque_Mtrx = Eigen::MatrixXd::Zero(12, 30);
    Eigen::VectorXd QP_Torque = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd QP_State = Eigen::VectorXd::Zero(30);

    Eigen::Matrix<qp_real, 30, 30> 	Cost_P; 
    Eigen::Matrix<qp_real, 30, 1> 	Cost_c; 
    Eigen::Matrix<qp_real, 24, 30> 	Inequality_G;
    Eigen::Matrix<qp_real, 24, 1> 	Inequality_h;
    Eigen::Matrix<qp_real, 6, 30> 	Equality_A; 
    Eigen::Matrix<qp_real, 6, 1> 	Equality_b; 

    Cost_P.block(0, 0, 30, 30) 		= CostFuntion_A_.block(0, 0, 30, 30);
    Cost_c.block(0, 0, 30, 1) 		= CostFuntion_B_.block(0, 0, 30, 1);
    
    Equality_A.block(0, 0, 6, 30)	= EqualityConstraint_A_.block(0, 0, 6, 30);	
    Equality_b.block(0, 0, 6, 1)	= EqualityConstraint_B_.block(0, 0, 6, 1);	

    Inequality_G.block(0, 0, 24, 30)= InequalityConstraint_A_.block(0, 0, 24, 30);
    Inequality_h.block(0, 0, 24, 1)	= InequalityConstraint_B_.block(0, 0, 24, 1);

    QP* WBC;
    qp_int n = 30;      qp_int m = 24;      qp_int p = 6;	
    WBC = QP_SETUP_dense(n, m, p, Cost_P.data(), Equality_A.data(), Inequality_G.data(), Cost_c.data(), Inequality_h.data(), Equality_b.data(), NULL, COLUMN_MAJOR_ORDERING);

    WBC->options->maxit  = 30;   
    WBC->options->reltol = 1e-3; 
    WBC->options->abstol  = 1e-3; 	

    qp_int ExitCode_ = QP_SOLVE(WBC);

    QP_State << 	WBC->x[0],  WBC->x[1],  WBC->x[2],  WBC->x[3],  WBC->x[4],  WBC->x[5],  WBC->x[6],  WBC->x[7],  WBC->x[8], 
                    WBC->x[9],  WBC->x[10], WBC->x[11], WBC->x[12], WBC->x[13], WBC->x[14], WBC->x[15], WBC->x[16], WBC->x[17], 
                    // ~ qddot / lambda ~ 
                    WBC->x[18], WBC->x[19], WBC->x[20], 
                    WBC->x[21], WBC->x[22], WBC->x[23],
                    WBC->x[24], WBC->x[25], WBC->x[26], 
                    WBC->x[27], WBC->x[28], WBC->x[29];
    return QP_State;
}


/////////// Equality Constraint //////////

Eigen::Matrix<double,6,30> PostureStablizer::CalcEqualityConstraint_A(  const Eigen::Matrix<double,3,4>& foot_pos, 
                                                                        const Eigen::Matrix<double,6,18>& inertia6,
                                                                        const bool swing_f[4])
{
    constexpr int numRows   = 6;
    constexpr int torsoCols = 18;
    constexpr int legCols   = 12;

    // 1) 결과 행렬: 6×30 제로 초기화
    Eigen::Matrix<double,6,30> EqCoA = Eigen::Matrix<double,6,30>::Zero();

    // 2) 다리 기여용 6×12 행렬
    Eigen::Matrix<double,6,12> WBC_J_Upper_Trs;
    WBC_J_Upper_Trs.setZero();

    // 단축키용 인덱스
    constexpr int FL = 0, FR = 1, RL = 2, RR = 3;
    constexpr int X = 0, Y = 1, Z = 2;

    // --- 힘 방향 (Fx, Fy, Fz) ---
    // Fx: -1, Fy: -1, Fz: -1
    for(int f=0; f<3; ++f){
        for(int leg=0; leg<4; ++leg){
            WBC_J_Upper_Trs(f, leg*3 + f) = -1.0;
        }
    }

    // --- 토크 항 ---
    // τx = y·Fz - z·Fy  → row 3
    // τy = z·Fx - x·Fz  → row 4
    // τz = x·Fy - y·Fx  → row 5
    for(int leg=0; leg<4; ++leg){
        double x = foot_pos(X,leg);
        double y = foot_pos(Y,leg);
        double z = foot_pos(Z,leg);
        int c = leg*3;

        // row 3: torque about X-axis
        WBC_J_Upper_Trs(3, c+1) =  z;   // Fz coeff
        WBC_J_Upper_Trs(3, c+2) = -y;   // -Fy coeff

        // row 4: torque about Y-axis
        WBC_J_Upper_Trs(4, c+0) = -z;   // -Fz coeff
        WBC_J_Upper_Trs(4, c+2) =  x;   // Fx coeff

        // row 5: torque about Z-axis
        WBC_J_Upper_Trs(5, c+0) =  y;   // Fy coeff
        WBC_J_Upper_Trs(5, c+1) = -x;   // -Fx coeff
    }

    // 3) EqCoA 에 몸통 관성 + 다리 기여 합치기
    EqCoA.block<6,18>(0, 0)  = inertia6;
    EqCoA.block<6,12>(0, torsoCols) = WBC_J_Upper_Trs;

    // 4) 스윙발은 지면 반력 0
    for(int leg = 0; leg < 4; ++leg) {
        double stance_multiplier = 1.0 - static_cast<double>(swing_f[leg]); 
        for(int j = 0; j < 3; ++j) {
            int col_index = torsoCols + leg * 3 + j; 
            EqCoA.block(0, col_index, numRows, 1) *= stance_multiplier;
        }
    }
    return EqCoA;
}
Eigen::Matrix<double,6,1 > PostureStablizer::CalcEqualityConstraint_B( Eigen::Matrix<double,6,1 >  Nonlinear6){
    Eigen::Matrix<double,6,1 > EqCoB = Eigen::Matrix<double,6,1 >::Zero();
    EqCoB = Nonlinear6;
    return EqCoB;		
}

////////// Inequality Constraint //////////

Eigen::Matrix<double,24,30> PostureStablizer::CalcInequalityConstraint_A( double MU )
{
    Eigen::Matrix<double,6,3>   Friction_Pyramid = Eigen::Matrix<double,6,3>   ::Zero();
    Eigen::Matrix<double,24,30> InEqCoA          = Eigen::Matrix<double,24,30> ::Zero();
    Friction_Pyramid << 1, 0, MU,        -1, 0, MU,        0, 1, MU,       0, -1, MU,         0, 0, 1,         0, 0,-1;
    for(int i = 0; i < 4; i++){
        InEqCoA.block(i*6, 18+i*3, 6, 3) = Friction_Pyramid.block(0, 0, 6, 3);
    }
    return InEqCoA;
}
Eigen::Matrix<double,24,1>  PostureStablizer::CalcInequalityConstraint_B( double H_Limit, double L_Limit, bool swing_f[4] )
{
    Eigen::Matrix<double,24,1> InEqCoB = Eigen::Matrix<double,24,1>::Zero(24);
    InEqCoB <<  0., 0., 0., 0.,  H_Limit*(1-swing_f[_FL_]), L_Limit*(1-swing_f[_FL_]),
                0., 0., 0., 0.,  H_Limit*(1-swing_f[_FR_]), L_Limit*(1-swing_f[_FR_]),
                0., 0., 0., 0.,  H_Limit*(1-swing_f[_RL_]), L_Limit*(1-swing_f[_RL_]),
                0., 0., 0., 0.,  H_Limit*(1-swing_f[_RR_]), L_Limit*(1-swing_f[_RR_]);
    return InEqCoB;
}

/////////// Cost Function //////////
Eigen::Matrix<double,30,30> PostureStablizer::CalcCostFuntion_A(Eigen::Matrix<double,18,18> J_)
{
    Eigen::Matrix<double,30,30> CoFuA           = Eigen::Matrix<double,30,30>::Zero();  
    Eigen::Matrix<double,18,30> A_Matrix        = Eigen::Matrix<double,18,30>::Zero();
    Eigen::Matrix<double,30,18> A_Matrix_Trs    = Eigen::Matrix<double,30,18>::Zero();
    A_Matrix.block(0, 0, 18, 18) = J_.block(0, 0, 18, 18);
    A_Matrix_Trs = A_Matrix.transpose();
    CoFuA	=	A_Matrix_Trs * A_Matrix;	
    return CoFuA;
}

Eigen::Matrix<double,30,1 > PostureStablizer::CalcCostFuntion_B(    
    Eigen::Matrix<double,18,1 > xddot_, Eigen::Matrix<double,18,18> J_dot_,     
    Eigen::Matrix<double,18,1 > qdot_,  Eigen::Matrix<double,18,18 > J_Trs_)
    {
    Eigen::Matrix<double,30,1 > CoFuB           = Eigen::Matrix<double,30,1 >::Zero();
    Eigen::Matrix<double,18,1 > b_Matrix_       = Eigen::Matrix<double,18,1 >::Zero();
    Eigen::Matrix<double,30,18> A_Matrix_Trs_   = Eigen::Matrix<double,30,18>::Zero();
    A_Matrix_Trs_.block(0, 0, 18, 18) = J_Trs_.block(0, 0, 18, 18);
    b_Matrix_ = xddot_ 	- J_dot_ * qdot_;	
    CoFuB =	(-1) * (A_Matrix_Trs_) * b_Matrix_;	
    return CoFuB;
}

    Eigen::Matrix<double,18,1 > PostureStablizer::GetXddotVector(  
        Eigen::Vector3d CoM_P, Eigen::Vector3d CoM_Des,        Eigen::Vector3d CoM_State, 
        Eigen::Vector3d CoM_D, Eigen::Vector3d CoM_dot_Des,    Eigen::Vector3d CoM_dot_State,
        Eigen::Vector3d RPY_P, Eigen::Vector3d RPY_Des,        Eigen::Vector3d RPY_State, 
        Eigen::Vector3d RPY_D, Eigen::Vector3d RPY_dot_Des,    Eigen::Vector3d RPY_dot_State)
        {
        double FeedForward_ = -9.81;
        Eigen::Matrix<double,18,1 > Xddot = Eigen::Matrix<double,18,1 >::Zero();
        Xddot(0) = PostureStablizer::CalcPDControl(CoM_P[_X_], CoM_Des[_X_], CoM_State[_X_], CoM_D[_X_], CoM_dot_Des[_X_], CoM_dot_State[_X_], 0);
        Xddot(1) = PostureStablizer::CalcPDControl(CoM_P[_Y_], CoM_Des[_Y_], CoM_State[_Y_], CoM_D[_Y_], CoM_dot_Des[_Y_], CoM_dot_State[_Y_], 0);
        Xddot(2) = PostureStablizer::CalcPDControl(CoM_P[_Z_], CoM_Des[_Z_], CoM_State[_Z_], CoM_D[_Z_], CoM_dot_Des[_Z_], CoM_dot_State[_Z_], FeedForward_);
        Xddot(3) = PostureStablizer::CalcPDControl(RPY_P[0], RPY_Des[0], RPY_State[0], RPY_D[0], RPY_dot_Des[0], RPY_dot_State[0], 0);
        Xddot(4) = PostureStablizer::CalcPDControl(RPY_P[1], RPY_Des[1], RPY_State[1], RPY_D[1], RPY_dot_Des[1], RPY_dot_State[1], 0);
        Xddot(5) = PostureStablizer::CalcPDControl(RPY_P[2], RPY_Des[2], RPY_State[2], RPY_D[2], RPY_dot_Des[2], RPY_dot_State[2], 0);
        return Xddot;
    }

    double PostureStablizer::CalcPDControl( double P_gain, double Desired,double State,  double D_gain, double Desired_dot, double State_dot, double FF){
        double PDcontrolled = P_gain * (Desired - State) + D_gain * (Desired_dot - State_dot) + FF;
        return PDcontrolled;
    }

