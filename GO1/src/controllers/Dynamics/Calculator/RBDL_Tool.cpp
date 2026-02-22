#include "RBDL_Tool.h"

RigidBodyDynamics::Model* RBDLRobot = nullptr;

RBDL_Tool::RBDL_Tool() {
}

RBDL_Tool::~RBDL_Tool() {
}
// 확인 완료
void RBDL_Tool::Init_RBDL(){
    RBDLRobot = new Model();
	Addons::URDFReadFromFile("../../model/urdf/go1.urdf", RBDLRobot, true, false);
	RBDLRobot->gravity = Eigen::Vector3d(0., 0., -9.81);

	ID_CoM = RBDLRobot->GetBodyId("base");	 
	ID_RR = RBDLRobot->GetBodyId("RR_foot"); 
	ID_RL = RBDLRobot->GetBodyId("RL_foot");
    ID_FR = RBDLRobot->GetBodyId("FR_foot");  
	ID_FL = RBDLRobot->GetBodyId("FL_foot"); 
}

// ----- Jacobian ----- //
Eigen::MatrixXd RBDL_Tool::CalcFootJacobian(const int leg_, const Eigen::VectorXd& q_state_) {
    Eigen::MatrixXd J_Full(3, 18);  // assuming full Jacobian is always 3x18
    Eigen::MatrixXd J_out = Eigen::MatrixXd::Zero(3, 3);

    int start_col = -1;
    unsigned int body_id = 0;

    switch (leg_) {
        case 0:
            body_id = ID_FL;
            start_col = 6;
            break;
        case 1:
            body_id = ID_FR;
            start_col = 9;
            break;
        case 2:
            body_id = ID_RL;
            start_col = 12;
            break;
        case 3:
            body_id = ID_RR;
            start_col = 15;
            break;
        default:
            return Eigen::MatrixXd::Zero(3, 3);
    }
    CalcPointJacobian(*RBDLRobot, q_state_, body_id, Eigen::Vector3d::Zero(), J_Full, true);
    J_out = J_Full.block(0, start_col, 3, 3);
    return J_out;
}

Eigen::MatrixXd RBDL_Tool::CalcWholeJacobian( Eigen::VectorXd q_state_){
    CalcPointJacobian(*RBDLRobot, q_state_, ID_FL,  Eigen::Vector3d::Zero(), J_FL_18, true);
    CalcPointJacobian(*RBDLRobot, q_state_, ID_FR,  Eigen::Vector3d::Zero(), J_FR_18, true);
    CalcPointJacobian(*RBDLRobot, q_state_, ID_RL,  Eigen::Vector3d::Zero(), J_RL_18, true);
    CalcPointJacobian(*RBDLRobot, q_state_, ID_RR,  Eigen::Vector3d::Zero(), J_RR_18, true);
    J_FL_.block(0, 0, 3, 3) = J_FL_18.block(0, 6,  3, 3);
    J_FR_.block(0, 0, 3, 3) = J_FR_18.block(0, 9,  3, 3);
    J_RL_.block(0, 0, 3, 3) = J_RL_18.block(0, 12, 3, 3);
    J_RR_.block(0, 0, 3, 3) = J_RR_18.block(0, 15, 3, 3);
    J_.block(0, 0, 3, 3) = I3.block(0, 0, 3, 3);
    J_.block(3, 3, 3, 3) = I3.block(0, 0, 3, 3);
    J_.block(6, 6, 3, 3) = J_FL_.block(0, 0, 3, 3);
    J_.block(9, 9, 3, 3) = J_FR_.block(0, 0, 3, 3);
    J_.block(12, 12, 3, 3) = J_RL_.block(0, 0, 3, 3);
    J_.block(15, 15, 3, 3) = J_RR_.block(0, 0, 3, 3);
    return J_;
}

Eigen::MatrixXd RBDL_Tool::CalcWholeJacobianTrans(Eigen::MatrixXd J_){
    J_legs.block(0, 0, 12, 12) = J_.block(6, 6, 12, 12);
    return J_legs.transpose();
}

Eigen::MatrixXd RBDL_Tool::CalcJacobianDot( const double L_0_,    const double L_1_,  const double L_2_, 
                                            const double q1,      const double q2,    const double q3,
                                            const double qdot1,   const double qdot2, const double qdot3, int leg_,
                                            Eigen::Matrix3d &result_Jacobian)
{
    double L0 = L_0_;
    double s1 = sin(q1), c1 = cos(q1), s2 = sin(q2), c2 = cos(q2), s23 = sin(q2 + q3), c23 = cos(q2 + q3);

    if(leg_ == FR || leg_ == RR)L0 = -L_0_;

    result_Jacobian(0,0) = 0;
    result_Jacobian(0,1) = L_1_*qdot2*s2 + L_2_*(qdot2 + qdot3)*s23 ;
    result_Jacobian(0,2) = L_2_*(qdot2 + qdot3)*s23 ;
    
    result_Jacobian(1,0) = -(qdot1*(L_1_*c2 + L_2_*c23)*s1) - c1*(L0*qdot1 + L_1_*qdot2*s2 + L_2_*(qdot2 + qdot3)*s23) ;
    result_Jacobian(1,1) = -(L_1_*qdot2*c2*s1) - L_2_*(qdot2 + qdot3)*c23*s1 - qdot1*c1*(L_1_*s2 + L_2_*s23) ;
    result_Jacobian(1,2) = -(L_2_*((qdot2 + qdot3)*c23*s1 + qdot1*c1*s23)) ;

    result_Jacobian(2,0) = qdot1*c1*(L_1_*c2 + L_2_*c23) - s1*(L0*qdot1 + L_1_*qdot2*s2 + L_2_*(qdot2 + qdot3)*s23) ;
    result_Jacobian(2,1) = c1*(L_1_*qdot2*c2 + L_2_*(qdot2 + qdot3)*c23) - qdot1*s1*(L_1_*s2 + L_2_*s23) ;
    result_Jacobian(2,2) = L_2_*((qdot2 + qdot3)*c1*c23 - qdot1*s1*s23) ;

    return result_Jacobian;
}

// ----- Inertia Matrix ----- //
Eigen::MatrixXd RBDL_Tool::GetWholeInertiaMatrix(Eigen::VectorXd q_state_)
{
    Eigen::MatrixXd InertiaMatrix = Eigen::MatrixXd::Zero(18, 18);
    CompositeRigidBodyAlgorithm(*RBDLRobot, q_state_, InertiaMatrix, true);
    return InertiaMatrix;
}

Eigen::MatrixXd RBDL_Tool::GetBodyInertiaMatrix(Eigen::VectorXd q_state_)
{
    Eigen::MatrixXd InertiaMatrix = Eigen::MatrixXd::Zero(18, 18);
    Eigen::MatrixXd Inertia_body = Eigen::MatrixXd::Zero(6, 18);
    CompositeRigidBodyAlgorithm(*RBDLRobot, q_state_, InertiaMatrix, true);
    Inertia_body.block(0, 0, 6, 18) = InertiaMatrix.block(0, 0, 6, 18);
    return Inertia_body;
}

Eigen::MatrixXd RBDL_Tool::GetLegInertiaMatrix(Eigen::VectorXd q_state_)
{
    Eigen::MatrixXd InertiaMatrix = Eigen::MatrixXd::Zero(18, 18);
    Eigen::MatrixXd Inertia_leg = Eigen::MatrixXd::Zero(12, 18);
    CompositeRigidBodyAlgorithm(*RBDLRobot, q_state_, InertiaMatrix, true);
    Inertia_leg.block(0, 0, 12, 18) = InertiaMatrix.block(6, 0, 12, 18);
    return Inertia_leg;
}

// ----- Nonlinear Vector ----- //
Eigen::VectorXd RBDL_Tool::GetWholeColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_)
{
    Eigen::VectorXd CGvector = Eigen::VectorXd::Zero(18);
    NonlinearEffects(*RBDLRobot, q_state_, q_dot_state_, CGvector, NULL);
    return CGvector;
}

Eigen::VectorXd RBDL_Tool::GetBodyColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_)
{
    Eigen::VectorXd CGvector = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd CGbody = Eigen::VectorXd::Zero(6);
    NonlinearEffects(*RBDLRobot, q_state_, q_dot_state_, CGvector, NULL);
    CGbody = CGvector.head(6);
    return CGbody;
}

Eigen::VectorXd RBDL_Tool::GetLegColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_)
{
    Eigen::VectorXd CGvector = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd CGleg = Eigen::VectorXd::Zero(12);
    NonlinearEffects(*RBDLRobot, q_state_, q_dot_state_, CGvector, NULL);
    CGleg = CGvector.tail(12);
    return CGleg;
}

// ----- URDF Info ----- // 
Eigen::Matrix3d RBDL_Tool::GetLinkInertia(const std::string& link_name) 
{
    // 모델에 해당 링크 이름이 존재하는지 확인
    if (RBDLRobot->mBodyNameMap.find(link_name) == RBDLRobot->mBodyNameMap.end()) {
        std::cerr << "오류: 링크 \"" << link_name << "\" 을(를) 모델에서 찾을 수 없습니다." << std::endl;
        return Eigen::Matrix3d::Zero();
    }
    // 해당 링크의 body id 획득
    unsigned int link_id = RBDLRobot->mBodyNameMap[link_name];
    // URDF로부터 읽어들인 inertia 값 반환
    return RBDLRobot->mBodies[link_id].mInertia;
}

Eigen::Vector3d RBDL_Tool::GetLinkCOM(const std::string& link_name) 
{
    // 모델에 해당 링크 이름이 존재하는지 확인
    if (RBDLRobot->mBodyNameMap.find(link_name) == RBDLRobot->mBodyNameMap.end()) {
        std::cerr << "오류: 링크 \"" << link_name << "\" 을(를) 모델에서 찾을 수 없습니다." << std::endl;
        return Eigen::Vector3d::Zero();
    }
    // 해당 링크의 body id 획득
    unsigned int link_id = RBDLRobot->mBodyNameMap[link_name];
    // URDF로부터 읽어들인 com 값 반환
    return RBDLRobot->mBodies[link_id].mCenterOfMass;
}

double RBDL_Tool::GetLinkMass(const std::string& link_name) 
{
    // 모델에 해당 링크 이름이 존재하는지 확인
    if (RBDLRobot->mBodyNameMap.find(link_name) == RBDLRobot->mBodyNameMap.end()) {
        std::cerr << "오류: 링크 \"" << link_name << "\" 을(를) 모델에서 찾을 수 없습니다." << std::endl;
        return 0.0;
    }
    // 해당 링크의 body id 획득
    unsigned int link_id = RBDLRobot->mBodyNameMap[link_name];
    // URDF로부터 읽어들인 무게(질량) 반환
    return RBDLRobot->mBodies[link_id].mMass;
}
