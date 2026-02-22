// SlipDetector.h
#ifndef SLIP_DETECTOR_H
#define SLIP_DETECTOR_H

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>

/// 상태별 가우시안 분포 파라미터 구조체
struct GaussParams {
    double mu_e;    // 위치 오차(e)의 평균
    double sigma_e; // 위치 오차(e)의 표준편차
    double mu_v;    // 속도(v)의 평균
    double sigma_v; // 속도(v)의 표준편차
};

/// 베이지안 융합 기반 슬립 감지 클래스
class SlipDetectorBayes {
public:
    /// 기본 생성자: 모든 다리에 동일한 파라미터를 채움
    SlipDetectorBayes();

    /**
     * 다리별 파라미터 지정 생성자
     * @param prior  P(S=1)
     * @param alpha  지수 스무딩 계수
     * @param threshold  슬립 판단 임계치
     * @param noSlipParams  각 다리의 비슬립 가우시안 파라미터 (size=4)
     * @param slipParams    각 다리의 슬립 가우시안 파라미터   (size=4)
     */
    SlipDetectorBayes(double prior,
                      double alpha,
                      double threshold,
                      const std::array<GaussParams,4>& noSlipParams,
                      const std::array<GaussParams,4>& slipParams);

    /// 다리별 목표 위치 설정
    void setDesiredPositions(const std::array<Eigen::Vector3d,4>& desired);

    /// 다리별 슬립 확률 및 상태 업데이트
    void updateLeg(int leg,
                   bool inStance,
                   const Eigen::Vector3d& footVel);

    /// 다리별 슬립 상태 반환
    bool isSlip(int leg) const;

    /// 다리별 현재 슬립 확률(belief) 반환
    double getBelief(int leg) const;

private:
    /// 1D 가우시안 확률밀도함수
    static double gauss1d(double x, double mu, double sigma);

    double prior_;    // P(S=1)
    double alpha_;    // 스무딩 계수
    double threshold_; // 슬립 판단 임계치

    // params_[state][leg]: state=0→비슬립, state=1→슬립
    std::array<std::array<GaussParams,4>,2> params_;

    std::array<double,4> beliefs_;   // 다리별 슬립 확률
    std::array<bool,4>   slipStates_; // 다리별 슬립 상태

    std::array<Eigen::Vector3d,4> desiredPos_; // 다리별 목표 발 위치


    std::array<std::array<size_t,4>,2>  sample_count_v_;
    std::array<std::array<double,4>,2>  mean_v_;
    std::array<std::array<double,4>,2>  M2_v_;
};

#endif // SLIP_DETECTOR_H
