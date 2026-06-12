/*
    Shubh Khandelwal
*/

#pragma once
#include "../spl/definitions.hpp"

namespace splexecutor
{

    template <size_t dof>
    struct DHParameters
    {

        std::array<double, dof> d;
        std::array<double, dof> a;
        std::array<double, dof> alpha;

        Eigen::Matrix4d transform(const double& theta, const size_t& i) const
        {
            Eigen::Matrix4d T;
            if (i >= dof || i < 0)
            {
                T << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
            } else
            {
                T << cos(theta), -sin(theta)*cos(alpha[i]),  sin(theta)*sin(alpha[i]), a[i]*cos(theta),
                     sin(theta),  cos(theta)*cos(alpha[i]), -cos(theta)*sin(alpha[i]), a[i]*sin(theta),
                              0,             sin(alpha[i]),             cos(alpha[i]),            d[i],
                              0,                         0,                         0,               1;
            }
            return T;
        }

    };

    class Executor
    {

        private:

            double dt;

        public:

            Executor(size_t frequency) : dt(1.0 / frequency)
            {}

    };

};