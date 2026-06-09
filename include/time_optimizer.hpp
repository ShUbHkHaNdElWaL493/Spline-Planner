/*
    Shubh Khandelwal
*/

#include "fitpack.hpp"
#include <gsl/gsl_integration.h>
#include "spline_definitions.hpp"

namespace splplanner
{
    class TimeOptimizer
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

            TimeOptimizer(const double max_vel, const double max_acc) : max_vel(max_vel), max_acc(max_acc) 
            {}

            std::pair<double, double> getOptimalKineticParameters(const std::vector<VectorRepresentation>& path) const
            {

                if (path.size() < 2) return {0.0, 0.0};

                size_t n = path.size();
                size_t num_dims = path[0].cols();

                Eigen::VectorXd u_eigen = Eigen::VectorXd::LinSpaced(n, 0.0, 1.0);
                std::vector<double> u(u_eigen.data(), u_eigen.data() + u_eigen.size());

                std::vector<std::vector<double>> p(num_dims, std::vector<double>(n));
                for (size_t i = 0; i < n; ++i)
                {
                    for (size_t j = 0; j < num_dims; ++j)
                    {
                        p[j][i] = path[i](j);
                    }
                }

                double smoothing = 0.005;
                std::vector<Spline> splines;
                splines.reserve(num_dims);
                for (size_t i = 0; i < num_dims; ++i)
                {
                    splines.push_back(splrep(u, p[i], smoothing));
                }
                IntegrandParams params{&splines};

                // Setup and Execute GSL Integration
                gsl_integration_workspace* workspace = gsl_integration_workspace_alloc(200);
                gsl_function F;
                F.function = &TimeOptimizer::integrand;
                F.params = &params;

                double distance = 0.0, error = 0.0;
                gsl_integration_qags(&F, 0.0, 1.0, 0, 1e-7, 200, workspace, &distance, &error);
                gsl_integration_workspace_free(workspace);

                double time = (distance / this->max_vel) + (this->max_vel / this->max_acc);

                return {distance, time};

            }
        
    };
}