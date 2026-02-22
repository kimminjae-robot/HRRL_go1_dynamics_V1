#ifndef DYNAMICSUTIL_H
#define DYNAMICSUTIL_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "enum.h"

class DynamicsUtil {
public:
    Eigen::VectorXd SetQState(const Eigen::VectorXd& q_vec);
    Eigen::VectorXd SetDQState(const Eigen::VectorXd& q_vec);
    Eigen::Matrix3d Quat2RotMatrix(std::vector<double>& quat);
    Eigen::Vector3d Quat2EulerClamped(const std::vector<double>& quat);
    Eigen::VectorXd ReMapLegOrder(const std::vector<double>& q_in);
    
private:
    Eigen::VectorXd z6 = Eigen::VectorXd::Zero(6);
};

#endif // DYNAMICSUTIL_H