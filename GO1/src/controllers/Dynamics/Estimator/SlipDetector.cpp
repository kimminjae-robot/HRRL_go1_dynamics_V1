// SlipDetector.cpp
#include "SlipDetector.h"

// —————————————————————————————————————————————
// 기본 생성자: 모든 다리에 동일한 기본 파라미터를 채움
SlipDetectorBayes::SlipDetectorBayes()
    : prior_(0.02), alpha_(0.9), threshold_(0.5)
{
    // 비슬립/슬립 기본 파라미터
    GaussParams noSlipDefault {0.0236469, 0.0238058, 0.389253, 1.60152};
    GaussParams slipDefault   {0.08,      0.1,      3.0,     2.0};

    // 모든 다리에 동일하게 복사
    for(int leg=0; leg<4; ++leg){
        params_[0][leg] = noSlipDefault;
        params_[1][leg] = slipDefault;
    }

    beliefs_.fill(prior_);
    slipStates_.fill(false);
    // sample_count_v_.fill(0);
    // mean_v_.fill(0.0);
    // M2_v_.fill(0.0);
    for(auto &row : sample_count_v_) row.fill(0);
    for(auto &row : mean_v_        ) row.fill(0.0);
    for(auto &row : M2_v_          ) row.fill(0.0);
}

// —————————————————————————————————————————————
// 파라미터 지정 생성자
SlipDetectorBayes::SlipDetectorBayes(double prior,
                                     double alpha,
                                     double threshold,
                                     const std::array<GaussParams,4>& noSlipParams,
                                     const std::array<GaussParams,4>& slipParams)
    : prior_(prior), alpha_(alpha), threshold_(threshold)
{
    // 다리별로 파라미터를 그대로 복사
    params_[0] = noSlipParams;
    params_[1] = slipParams;

    beliefs_.fill(prior_);
    slipStates_.fill(false);
    // sample_count_v_.fill(0);
    // mean_v_.fill(0.0);
    // M2_v_.fill(0.0);
    for (auto &row : sample_count_v_) row.fill(0);
    for (auto &row : mean_v_       ) row.fill(0.0);
    for (auto &row : M2_v_         ) row.fill(0.0);
}

// —————————————————————————————————————————————
void SlipDetectorBayes::setDesiredPositions(const std::array<Eigen::Vector3d,4>& desired) {
    desiredPos_ = desired;
    beliefs_.fill(prior_);
    slipStates_.fill(false);
    // sample_count_v_.fill(0);
    // mean_v_.fill(0.0);
    // M2_v_.fill(0.0);
    for (auto &row : sample_count_v_) row.fill(0);
    for (auto &row : mean_v_       ) row.fill(0.0);
    for (auto &row : M2_v_         ) row.fill(0.0);
}


void SlipDetectorBayes::updateLeg(int leg,
    bool inStance,
    const Eigen::Vector3d& footVel)
{
    // double v = footVel.norm();
    double v = std::hypot(footVel.x(), footVel.y());
    if (!inStance) {
    beliefs_[leg]    = 0.0;
    slipStates_[leg] = false;
    return;
    }

    // 1) Static posterior 계산
    double p0_static = gauss1d(v,
    params_[0][leg].mu_v,
    params_[0][leg].sigma_v) * (1.0 - prior_);
    double p1_static = gauss1d(v,
    params_[1][leg].mu_v,
    params_[1][leg].sigma_v) * prior_;
    double pSlip     = p1_static / (p0_static + p1_static + 1e-12);

    // 2) Belief & state 업데이트
    beliefs_[leg]    = alpha_ * beliefs_[leg] + (1.0 - alpha_) * pSlip;
    slipStates_[leg] = (beliefs_[leg] > threshold_);

    // 3) 해당 state의 통계(Welford) 업데이트
    int state = slipStates_[leg] ? 1 : 0;
    auto& sc = sample_count_v_[state][leg];
    auto& mv = mean_v_[state][leg];
    auto& m2 = M2_v_[state][leg];

    ++sc;
    double delta  = v - mv;
    mv           += delta / sc;
    double delta2 = v - mv;
    m2           += delta * delta2;

    // 4) 상태별 표준편차 계산
    double std_ns = (sample_count_v_[0][leg] > 1)
    ? std::sqrt(M2_v_[0][leg] / (sample_count_v_[0][leg] - 1))
    : params_[0][leg].sigma_v;
    double std_s  = (sample_count_v_[1][leg] > 1)
    ? std::sqrt(M2_v_[1][leg] / (sample_count_v_[1][leg] - 1))
    : params_[1][leg].sigma_v;

    // double mean_nonSlip = [0.18,    0.18,   0.18,   0.18];
    // double std_nonSlip  = [0.2,     0.2,    0.2,    0.2];
    // double mean_Slip    = [0.25,    0.25,   0.25,   0.25];
    // double std_Slip     = [0.5,    0.5,    0.5,    0.5];
    static const std::array<double,4> MEAN_NS = {0.00, 0.00, 0.00, 0.00};
    static const std::array<double,4> STD_NS  = {0.35, 0.35, 0.35, 0.35};
    static const std::array<double,4> MEAN_S  = {0.40, 0.40, 0.40, 0.40};
    static const std::array<double,4> STD_S   = {1.70, 1.70, 1.70, 1.70};
    double mean_nonSlip = MEAN_NS[leg];
    double std_nonSlip  = STD_NS [leg];
    double mean_slip    = MEAN_S [leg];
    double std_slip     = STD_S  [leg];

    // 5) 동적 분포로 posterior 재계산
    double p0_dyn = gauss1d(v, mean_nonSlip, std_nonSlip) * (1.0 - prior_);
    double p1_dyn = gauss1d(v, mean_slip,     std_slip)     * prior_;
    double pSlip_dyn = p1_dyn / (p0_dyn + p1_dyn + 1e-12);
    
    slipStates_[leg] = (pSlip_dyn > threshold_);
    beliefs_[leg]    = pSlip_dyn;  // 혹은 그대로 스무딩에 넣고 싶으면 이 값을 사용

    // beliefs_[leg] = alpha_*beliefs_[leg] + (1-alpha_)*pSlip_dyn;
    // slipStates_[leg] = (beliefs_[leg] > threshold_);

    // std::cout 
    // << "Leg " << leg
    // << " [" << (state ? "SLIP" : "NOSLIP") << "]"
    // << " μ_ns=" << mean_nonSlip
    // << " σ_ns=" << std_nonSlip
    // << "  μ_s=" << mean_slip
    // << " σ_s=" << std_slip
    // << "  pSlip_dyn=" << pSlip_dyn
    // << std::endl;
}

// —————————————————————————————————————————————
bool  SlipDetectorBayes::isSlip(int leg)    const { return slipStates_[leg]; }
double SlipDetectorBayes::getBelief(int leg) const { return beliefs_[leg]; }

// 1D 가우시안

double SlipDetectorBayes::gauss1d(double x, double mu, double sigma) {
    const double inv_sqrt_2pi = 1.0 / std::sqrt(2 * M_PI);
    double z = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-0.5 * z * z);
}
