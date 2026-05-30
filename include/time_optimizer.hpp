/*
    Shubh Khandelwal
*/

#include <cmath>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_errno.h>
#include <iostream>
#include "spline_definitions.hpp"
#include <unsupported/Eigen/Splines>
#include <utility>

class TimeOptimizer
{

    private:

        double max_vel, max_acc;

    public:

        TimeOptimizer(const double max_vel, const double max_acc) : max_vel(max_vel), max_acc(max_acc) 
        {}

        std::pair<double, double> getOptimalKineticParameters(const std::vector<SplineVector>& path) const
        {
        
            if (path.size() < 2) 
            {
                return {0.0, 0.0};
            }

            size_t n = path.size();
            Eigen::VectorXd u = Eigen::VectorXd::LinSpaced(n, 0.0, 1.0);

            double distance = 0.0;
            double time = (distance / this->max_vel) + (this->max_vel / this->max_acc);
            return {distance, time};
        }
    
};