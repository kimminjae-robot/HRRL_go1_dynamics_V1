#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS 
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h> 
#include "enum.h"
extern RigidBodyDynamics::Model *RBDLRobot;		  

using namespace RigidBodyDynamics;

class RBDL_Tool
{ 
public:
    RBDL_Tool();
    ~RBDL_Tool();
    void Init_RBDL();
    Eigen::MatrixXd CalcFootJacobian(const int leg_, const Eigen::VectorXd& q_state_);
    Eigen::MatrixXd CalcWholeJacobian( Eigen::VectorXd q_state_);
    Eigen::MatrixXd CalcWholeJacobianTrans(Eigen::MatrixXd J_);
    Eigen::MatrixXd CalcJacobianTrans(int leg_, Eigen::VectorXd q_state_);
    Eigen::MatrixXd CalcJacobianDot(const double L_0_, const double L_1_, const double L_2_, const double q1, const double q2, const double q3, const double qdot1,   const double qdot2, const double qdot3, int leg_, Eigen::Matrix3d &result_Jacobian);

    Eigen::MatrixXd GetWholeInertiaMatrix(Eigen::VectorXd q_state_);
    Eigen::MatrixXd GetBodyInertiaMatrix(Eigen::VectorXd q_state_);
    Eigen::MatrixXd GetLegInertiaMatrix(Eigen::VectorXd q_state_);

    Eigen::VectorXd GetWholeColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_);
    Eigen::VectorXd GetBodyColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_);
    Eigen::VectorXd GetLegColiolisAndGravityVector(Eigen::VectorXd q_state_, Eigen::VectorXd q_dot_state_);
    
    Eigen::Matrix3d GetLinkInertia(const std::string& link_name);
    double GetLinkMass(const std::string& link_name) ;
    Eigen::Vector3d GetLinkCOM(const std::string& link_name) ;

    
private:
    int FL = 0;
    int FR = 1;
    int RL = 2;
    int RR = 3;
    
    int ID_CoM;
    int ID_FL;  int ID_FR;  int ID_RL;  int ID_RR;
    Eigen::VectorXd q_state_    = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd q_dot_state_= Eigen::VectorXd::Zero(18);

    // 3 3
    Eigen::MatrixXd I3          = Eigen::MatrixXd::Identity(3, 3); 
    Eigen::MatrixXd Z3          = Eigen::MatrixXd::Zero(3, 3); 
        
    Eigen::MatrixXd J_FL_	    = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_FR_	    = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_RL_	    = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_RR_	    = Eigen::MatrixXd::Zero(3, 3);

    Eigen::MatrixXd J_Dot_FL 	= Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_Dot_FR 	= Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_Dot_RL 	= Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd J_Dot_RR 	= Eigen::MatrixXd::Zero(3, 3);

    // 3 18
    Eigen::MatrixXd J_FL_18	    = Eigen::MatrixXd::Zero(3, 18);
    Eigen::MatrixXd J_FR_18	    = Eigen::MatrixXd::Zero(3, 18);
    Eigen::MatrixXd J_RL_18	    = Eigen::MatrixXd::Zero(3, 18);
    Eigen::MatrixXd J_RR_18	    = Eigen::MatrixXd::Zero(3, 18);

    // 12 12 
    Eigen::MatrixXd J_legs  = Eigen::MatrixXd::Zero(12, 12);


    // 18 18
    Eigen::MatrixXd J_          = Eigen::MatrixXd::Zero(18, 18);
    Eigen::MatrixXd J_Dot_ 	    = Eigen::MatrixXd::Zero(18, 18);


};

