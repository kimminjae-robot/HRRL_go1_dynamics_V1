#include "Kinematics.h"

Kinematics::Kinematics() {
}

Kinematics::~Kinematics() {
}

void Kinematics::legFK( const double _q1,const double _q2, const double _q3,  const int _leg,  
                        double _L0, const double _L1, const double _L2, 
                        const double Robot_width, const double Robot_pside, 
                        const Eigen::Matrix3d &R_Mtrx_, Eigen::MatrixXd &result_xyz) {
    double offset = _L0;
    // 다리 위치에 따른 _L0 값 조정
    if (_leg == _FR_ || _leg == _RR_) _L0 = -_L0;

    Eigen::VectorXd Tmp_result_xyz = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Tmp_result_xyz2 = Eigen::VectorXd::Zero(3);

    // Forward Kinematics 계산
    Tmp_result_xyz(0) = -(_L1 * sin(_q2)) - _L2 * sin(_q2 + _q3);
    Tmp_result_xyz(1) = _L0 * cos(_q1) + (_L1 * cos(_q2) + _L2 * cos(_q2 + _q3)) * sin(_q1);
    Tmp_result_xyz(2) = -(cos(_q1) * (_L1 * cos(_q2) + _L2 * cos(_q2 + _q3))) + _L0 * sin(_q1);

    // 몸체 중심에서 다리 위치 보정
    if (_leg == _FL_) {
        Tmp_result_xyz(0) += Robot_width / 2;
        Tmp_result_xyz(1) += (Robot_pside / 2 - offset);
    } else if (_leg == _FR_) {
        Tmp_result_xyz(0) += Robot_width / 2;
        Tmp_result_xyz(1) -= (Robot_pside / 2 - offset);
    } else if (_leg == _RL_) {
        Tmp_result_xyz(0) -= Robot_width / 2;
        Tmp_result_xyz(1) += (Robot_pside / 2 - offset);
    } else if (_leg == _RR_) {
        Tmp_result_xyz(0) -= Robot_width / 2;
        Tmp_result_xyz(1) -= (Robot_pside / 2 - offset);
    }
    // 회전 행렬을 사용한 좌표 변환
    Tmp_result_xyz2 = R_Mtrx_ * Tmp_result_xyz;
    // 결과 저장
    result_xyz( _X_,_leg) = Tmp_result_xyz2(0);
    result_xyz( _Y_,_leg) = Tmp_result_xyz2(1);
    result_xyz( _Z_,_leg) = Tmp_result_xyz2(2);
}

void Kinematics::legIK(const int _leg, const double FootTraj[3][4], const double Robot_pside, const double _L0, const double _L1, const double _L2,  double result[4][3]) {
    double hip_pos = Robot_pside / 2 - _L0;
    bool isRight = (_leg == 1 || _leg == 3);
    double effective_y = isRight ? -FootTraj[_Y_][_leg] - hip_pos : FootTraj[_Y_][_leg] - hip_pos;
    
    double A = atan2(effective_y, -FootTraj[_Z_][_leg]);
    double B = atan2( sqrt(pow(-FootTraj[_Z_][_leg], 2) + pow(effective_y, 2) - pow(_L0, 2) ), _L0 );
    double C = sqrt( pow(-FootTraj[_Z_][_leg], 2) + pow(effective_y, 2) - pow(_L0, 2) + pow(FootTraj[_X_][_leg], 2) );
    double beta = atan2( FootTraj[_X_][_leg], sqrt( pow(-FootTraj[_Z_][_leg], 2) + pow(effective_y, 2) - pow(_L0, 2) ) );
    
    if (isRight)
        result[_leg][0] = -(B - (90 * DEG2RAD - A));
    else
        result[_leg][0] = B - (90 * DEG2RAD - A);
    
    result[_leg][1] = -(180 * DEG2RAD - acos( (pow(_L1, 2) + pow(_L2, 2) - pow(C, 2))/(2 * _L1 * _L2) ));
    result[_leg][2] = acos( (pow(_L1, 2) + pow(C, 2) - pow(_L2, 2))/(2 * _L1 * C) ) - beta;
}

double Kinematics::calcKinematicHeight(Eigen::MatrixXd fp, double filter_hz, double dt)  
{
    double Kinematic_height;
    // 몸통 높이 추정: 가장 멀리 뻗은 다리 선택 (Z축 값 비교)
    if ((-fp(_Z_, _FL_)  > -fp(_Z_, _FR_) ) &&
        (-fp(_Z_, _FL_)  > -fp(_Z_, _RL_) ) &&
        (-fp(_Z_, _FL_)  > -fp(_Z_, _RR_) ))
    {
        Kinematic_height = -fp(_Z_, _FL_);
    }
    else if ((-fp(_Z_, _FR_)  > -fp(_Z_, _FL_) ) &&
             (-fp(_Z_, _FR_)  > -fp(_Z_, _RL_) ) &&
             (-fp(_Z_, _FR_)  > -fp(_Z_, _RR_) ))
    {
        Kinematic_height = -fp(_Z_, _FR_);
    }
    else if ((-fp(_Z_, _RL_)  > -fp(_Z_, _FL_) ) &&
             (-fp(_Z_, _RL_)  > -fp(_Z_, _FR_) ) &&
             (-fp(_Z_, _RL_)  > -fp(_Z_, _RR_) ))
    {
        Kinematic_height = -fp(_Z_, _RL_);
    }
    else if ((-fp(_Z_, _RR_)  > -fp(_Z_, _FL_) ) &&
             (-fp(_Z_, _RR_)  > -fp(_Z_, _FR_) ) &&
             (-fp(_Z_, _RR_)  > -fp(_Z_, _RL_) ))
    {
        Kinematic_height = -fp(_Z_, _RR_);
    }

    A_z = Kinematic_height;
    LPF_z = (1 - dt * 2 * PI_ * filter_hz) * LPF_current + (dt * 2 * PI_ * filter_hz) * A_z;
    LPF_current = LPF_z;

    Kinematic_height = LPF_z;
    
    return Kinematic_height;
}

