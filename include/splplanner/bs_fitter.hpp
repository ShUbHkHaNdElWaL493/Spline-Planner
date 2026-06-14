/*
    Shubh Khandelwal
*/

#pragma once
#include "fitpack.hpp"
#include "spl/definitions.hpp"

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

            std::vector<Spline> fitSpline(const std::vector<spl::VectorRepresentation>& path) const
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

            std::pair<std::vector<double>, spl::Trajectory> evaluate(const std::vector<Spline>& splines, const double &t) const
            {

                double t_inverse_1 = 1.0 / t;
                double t_inverse_2 = t_inverse_1 * t_inverse_1;
                double t_inverse_3 = t_inverse_1 * t_inverse_2;
                int num_steps = t * frequency;
                size_t num_dims = splines.size();

                Eigen::VectorXd u_eigen = Eigen::VectorXd::LinSpaced(num_steps, 0.0, t);
                std::vector<double> u_eigen_vector(u_eigen.data(), u_eigen.data() + u_eigen.size());
                u_eigen = u_eigen / t;
                std::vector<double> u(u_eigen.data(), u_eigen.data() + u_eigen.size());

                std::vector<spl::VectorRepresentation> pos(num_steps), vel(num_steps), acc(num_steps), jrk(num_steps);
                for (size_t i = 0; i < num_steps; ++i)
                {
                    spl::VectorRepresentation pos_val(num_dims), vel_val(num_dims), acc_val(num_dims), jrk_val(num_dims);
                    for (size_t j = 0; j < num_dims; ++j)
                    {
                        pos_val(j) = splev(splines[j], u[i], 0);
                        vel_val(j) = splev(splines[j], u[i], 1) * t_inverse_1;
                        acc_val(j) = splev(splines[j], u[i], 2) * t_inverse_2;
                        jrk_val(j) = splev(splines[j], u[i], 3) * t_inverse_3;
                    }
                    pos[i] = pos_val;
                    vel[i] = vel_val;
                    acc[i] = acc_val;
                    jrk[i] = jrk_val;
                }

                return {u_eigen_vector, {pos, vel, acc, jrk}};

            }

    };
}