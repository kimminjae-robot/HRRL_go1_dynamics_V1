// KalmanFilter.cpp
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(int state_dim, int input_dim, int meas_dim)
    : state_dim(state_dim),
      input_dim(input_dim),
      meas_dim(meas_dim),
      x(Eigen::VectorXd::Zero(state_dim)),
      P(Eigen::MatrixXd::Identity(state_dim, state_dim)),
      A(Eigen::MatrixXd::Identity(state_dim, state_dim)),
      B(Eigen::MatrixXd::Zero(state_dim, input_dim)),
      C(Eigen::MatrixXd::Zero(meas_dim, state_dim)),
      D(Eigen::MatrixXd::Zero(meas_dim, input_dim)),
      Q(Eigen::MatrixXd::Identity(state_dim, state_dim)),
      R(Eigen::MatrixXd::Identity(meas_dim, meas_dim)) {}

void KalmanFilter::predict(const Eigen::VectorXd& u) {
    // x = A*x + B*u
    x.noalias() = A * x;
    x.noalias() += B * u;

    // P = A*P*Aᵀ + Q
    P.noalias() = A * P * A.transpose();
    P += Q;
}

void KalmanFilter::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u) {
    // S = C*P*Cᵀ + R
    Eigen::MatrixXd S(meas_dim, meas_dim);
    S.noalias() = C * P * C.transpose();
    S += R;

    // K = P*Cᵀ * S⁻¹  (LDLT 분해 이용)
    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    Eigen::MatrixXd PCt = P * C.transpose();
    Eigen::MatrixXd K = PCt * ldlt.solve(Eigen::MatrixXd::Identity(meas_dim, meas_dim));

    // x = x + K*(y - C*x)
    Eigen::VectorXd e = y - (C * x + D * u);
    // Eigen::VectorXd e = y - (C * x);
    x.noalias() += K * e;

    // P = (I - K*C)*P
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    P.noalias() = (I - K * C) * P;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return x;
}

void KalmanFilter::setA(const Eigen::MatrixXd& A_in) { A = A_in; }
void KalmanFilter::setB(const Eigen::MatrixXd& B_in) { B = B_in; }
void KalmanFilter::setC(const Eigen::MatrixXd& C_in) { C = C_in; }
void KalmanFilter::setD(const Eigen::MatrixXd& D_in) { D = D_in; }
void KalmanFilter::setQ(const Eigen::MatrixXd& Q_in) { Q = Q_in; }
void KalmanFilter::setR(const Eigen::MatrixXd& R_in) { R = R_in; }
void KalmanFilter::setx0(const Eigen::VectorXd& x0)     { x = x0; }
void KalmanFilter::setP0(const Eigen::MatrixXd& P0)     { P = P0; }
