/*
    Shubh Khandelwal
*/

#pragma once
#include "fitpack.hpp"
#include <gsl/gsl_integration.h>
#include "spline_definitions.hpp"

namespace splplanner
{
    class KinematicsOptimizer
    {

        private:

            double max_vel, max_acc;

            struct IntegrandParams 
            {
                const std::vector<Spline>* splines;
            };

            static double integrand(double u, void* params) 
            {
                IntegrandParams* p = static_cast<IntegrandParams*>(params);
                double safe_u = std::max(0.0, std::min(1.0, u));
                double result = 0.0;
                for (const Spline& spline : *(p->splines))
                {
                    double dx = splplanner::splev(spline, u, 1);
                    result += dx * dx;
                }
                return std::sqrt(result);
            }

        public:

            KinematicsOptimizer(const double max_vel, const double max_acc) : max_vel(max_vel), max_acc(max_acc)
            {}

            std::pair<double, double> getOptimalKinematicsParameters(const std::vector<Spline>& splines) const
            {

                IntegrandParams params{&splines};

                gsl_integration_workspace* workspace = gsl_integration_workspace_alloc(200);
                gsl_function F;
                F.function = &KinematicsOptimizer::integrand;
                F.params = &params;

                double distance = 0.0, error = 0.0;
                gsl_integration_qags(&F, 0.0, 1.0, 0, 1e-7, 200, workspace, &distance, &error);
                gsl_integration_workspace_free(workspace);

                double time = (distance / this->max_vel) + (this->max_vel / this->max_acc);

                return {distance, time};

            }

    };
}