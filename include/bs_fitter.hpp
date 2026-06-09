/*
    Shubh Khandelwal
*/

#pragma once
#include "fitpack.hpp"
#include "spline_definitions.hpp"

namespace splplanner
{
    class BSFitter
    {

        private:

            double s;
            size_t frequency;
        
        public:

            BSFitter(const size_t& frequency, const double& s) : frequency(frequency), s(s)
            {}

            std::vector<Spline> fitSpline(const std::vector<VectorRepresentation>& path) const
            {

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

                std::vector<Spline> splines;
                splines.reserve(num_dims);
                for (size_t i = 0; i < num_dims; ++i)
                {
                    splines.push_back(splrep(u, p[i], this->s));
                }

                return splines;

            }
        
    };
}