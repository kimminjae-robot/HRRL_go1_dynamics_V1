#ifndef POSTURESTABILIZER_H
#define POSTURESTABILIZER_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "qpSWIFT.h" 
#include "enum.h"
class PostureStablizer
{ 
public:
    PostureStablizer();
    ~PostureStablizer();
    Eigen::VectorXd WBC_QP( Eigen::MatrixXd CostFuntion_A_,         Eigen::VectorXd CostFuntion_B_, 
                            Eigen::MatrixXd EqualityConstraint_A_,  Eigen::VectorXd EqualityConstraint_B_, 
                            Eigen::MatrixXd InequalityConstraint_A_,Eigen::VectorXd InequalityConstraint_B_,
                            Eigen::MatrixXd J_legs_Trs_,            Eigen::MatrixXd Inertia12);

    Eigen::Matrix<double,6,30> CalcEqualityConstraint_A( const Eigen::Matrix<double,3,4>& foot_pos,  const Eigen::Matrix<double,6,18>& inertia6, const bool swing_f[4]);
    Eigen::Matrix<double,6,1 > CalcEqualityConstraint_B( Eigen::Matrix<double,6,1 > Nonlinear6);

    Eigen::Matrix<double,24,30> CalcInequalityConstraint_A( double MU );
    Eigen::Matrix<double,24,1>  CalcInequalityConstraint_B( double H_Limit, double L_Limit, bool swing_f[4] );

    Eigen::Matrix<double,18,1 > GetXddotVector( 
        Eigen::Vector3d CoM_P, Eigen::Vector3d CoM_Des,        Eigen::Vector3d CoM_State, 
        Eigen::Vector3d CoM_D, Eigen::Vector3d CoM_dot_Des,    Eigen::Vector3d CoM_dot_State,
        Eigen::Vector3d RPY_P, Eigen::Vector3d RPY_Des,        Eigen::Vector3d RPY_State, 
        Eigen::Vector3d RPY_D, Eigen::Vector3d RPY_dot_Des,    Eigen::Vector3d RPY_dot_State); 

    double CalcPDControl( double P_gain, double Desired,double State,  double D_gain, double Desired_dot, double State_dot, double FF);

    Eigen::Matrix<double,30,30> CalcCostFuntion_A( Eigen::Matrix<double,18,18>J_ );

    Eigen::Matrix<double,30,1 > CalcCostFuntion_B( Eigen::Matrix<double,18,1 > xddot_, Eigen::Matrix<double,18,18> J_dot_,   
                                                   Eigen::Matrix<double,18,1 > qdot_,  Eigen::Matrix<double,18,18 > J_Trs_);
    
    
private:


};

#endif // POSTURESTABILIZER_H
