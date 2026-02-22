#include "GeneralCalculator.h"

GeneralCalculator::GeneralCalculator() 
{
}

double GeneralCalculator::LowPassFilter(double input, double prev_output, double cutoff_frequency, double sampling_time) {
    // 컷오프 주파수에 따른 필터 계수 계산
    double rc = 1.0 / (2.0 * PI * cutoff_frequency);
    double alpha = sampling_time / (rc + sampling_time);

    // 로우패스 필터 공식 적용
    double output = alpha * input + (1.0 - alpha) * prev_output;

    return output;
}

double GeneralCalculator::logisticFunction( const double k, const double x, const double mid) {
    return 1.0 / (1.0 + std::exp(-k * (x - mid)));
}

double GeneralCalculator::diff(double newone, double oldone, double dt){
    return ( newone - oldone ) /dt;
}

double GeneralCalculator::quintic(double t) {
    return 6*t*t*t*t*t - 15*t*t*t*t + 10*t*t*t;
}

double GeneralCalculator::dquintic(double t) {
    return (30*t*t*t*t - 60*t*t*t + 30*t*t);
}
