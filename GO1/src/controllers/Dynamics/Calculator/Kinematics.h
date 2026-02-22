#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "../enum.h"

class Kinematics
{ 
public:
    Kinematics();
    ~Kinematics();
    // Forward Kinematics
    void legFK( const double _q1,const double _q2, const double _q3, 
                const int _leg,  double _L0, const double _L1, const double _L2, 
                const double Robot_width, const double Robot_pside, const Eigen::Matrix3d &R_Mtrx_ , Eigen::MatrixXd &result_xyz) ;
    // Inverse Kinematics
    void legIK(const int _leg, const double FootTraj[3][4], const double Robot_pside, const double _L0, const double _L1, const double _L2,  double result[4][3]) ;
    double calcKinematicHeight(Eigen::MatrixXd fp, double filter_hz, double dt) ; 
private:
    double A_z, LPF_z, LPF_current;

};

#endif // KINEMATICS_H
