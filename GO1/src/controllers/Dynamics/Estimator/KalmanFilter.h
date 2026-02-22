// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    // 생성자: state_dim = 상태 차원, input_dim = 입력 차원, meas_dim = 관측 차원
    KalmanFilter(int state_dim, int input_dim, int meas_dim);

    // 예측 및 갱신 함수
    void predict(const Eigen::VectorXd& u);
    void update(const Eigen::VectorXd& y,const Eigen::VectorXd& u);

    // 상태 접근자
    Eigen::VectorXd getState() const;

    // 설정자
    void setA(const Eigen::MatrixXd& A_in);
    void setB(const Eigen::MatrixXd& B_in);
    void setC(const Eigen::MatrixXd& C_in);
    void setD(const Eigen::MatrixXd& D_in);
    void setQ(const Eigen::MatrixXd& Q_in);
    void setR(const Eigen::MatrixXd& R_in);
    void setx0(const Eigen::VectorXd& x0);
    void setP0(const Eigen::MatrixXd& P0);

private:
    int state_dim;
    int input_dim;
    int meas_dim;

    Eigen::VectorXd x;       // 상태 벡터
    Eigen::MatrixXd P;       // 오차 공분산
    Eigen::MatrixXd A, B, C, D; // 시스템/입력/관측/피드포워드 행렬
    Eigen::MatrixXd Q, R;    // 프로세스 노이즈, 관측 노이즈 공분산
};

#endif // KALMANFILTER_H
