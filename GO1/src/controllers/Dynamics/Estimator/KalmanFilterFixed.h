// KalmanFilterFixed.h
#pragma once
#include <Eigen/Dense>
#include <cassert>

// N: state_dim, U: input_dim, M: meas_dim
template<int N, int U, int M>
class KalmanFilterFixed {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using VecN  = Eigen::Matrix<double, N, 1>;
  using VecU  = Eigen::Matrix<double, U, 1>;
  using VecM  = Eigen::Matrix<double, M, 1>;

  using MatNN = Eigen::Matrix<double, N, N>;
  using MatNU = Eigen::Matrix<double, N, U>;
  using MatMN = Eigen::Matrix<double, M, N>;
  using MatMU = Eigen::Matrix<double, M, U>;
  using MatMM = Eigen::Matrix<double, M, M>;
  using MatNM = Eigen::Matrix<double, N, M>;

  // states
  VecN  x;
  MatNN P;

  // system
  MatNN A, Q;
  MatNU B;
  MatMN C;
  MatMU D;
  MatMM R;

  // caches (no re-allocation)
  MatMN CP;   // C*P
  MatMM S;    // innovation covariance
  MatNM K;    // Kalman gain
  VecM  e;    // residual

  KalmanFilterFixed() {
    x.setZero();
    P.setIdentity();

    A.setIdentity();
    B.setZero();
    C.setZero();
    D.setZero();

    Q.setIdentity();
    R.setIdentity();

    CP.setZero();
    S.setZero();
    K.setZero();
    e.setZero();
  }

  // --- setters (기존 API 유지용) ---
  inline void setA(const MatNN& Ain) { A = Ain; }
  inline void setB(const MatNU& Bin) { B = Bin; }
  inline void setC(const MatMN& Cin) { C = Cin; }
  inline void setD(const MatMU& Din) { D = Din; }
  inline void setQ(const MatNN& Qin) { Q = Qin; }
  inline void setR(const MatMM& Rin) { R = Rin; }

  inline void setState(const VecN& xin) { x = xin; }
  inline void setCov(const MatNN& Pin)  { P = Pin; }


  // --- core ---
  inline void predict(const VecU& u) {
    // x = A x + B u
    x.noalias() = A * x;
    x.noalias() += B * u;

    // P = A P A^T + Q
    // (임시 최소화를 위해 CP를 활용하고 싶다면 추가 최적화 가능)
    P = A * P * A.transpose();
    P += Q;
  }

  inline void update(const VecM& y, const VecU& u) {
    // CP = C*P
    CP.noalias() = C * P;

    // S = CP*C^T + R
    S.noalias() = CP * C.transpose();
    S.noalias() += R;

    // e = y - (C*x + D*u)
    e.noalias() = y;
    e.noalias() -= C * x;
    e.noalias() -= D * u;

    // K^T = S^{-1} * CP  (solve(I) 금지)
    Eigen::LDLT<MatMM> ldlt(S);
    K = ldlt.solve(CP).transpose();

    // x = x + K*e
    x.noalias() += K * e;

    // P = P - K*(C*P)  (Joseph form 필요하면 여기서 바꿀 수 있음)
    P.noalias() -= K * CP;
  }

  inline VecN getState() const { return x; }  // 기존 호환 위해 복사 반환
};