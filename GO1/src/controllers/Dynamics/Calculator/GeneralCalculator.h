#ifndef GENERALCALCULATOR_H
#define GENERALCALCULATOR_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "enum.h"

class GeneralCalculator {
public:
    GeneralCalculator();
    double LowPassFilter(double input, double prev_output, double cutoff_frequency, double sampling_time) ;
    static double logisticFunction( const double k, const double x, const double mid);
    double diff(double newone, double oldone, double dt);
    static double quintic(double t) ;
    static double dquintic(double t) ;
private:
};

#endif // GENERALCALCULATOR_H
