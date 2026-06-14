/*
    Shubh Khandelwal
*/

#pragma once
#include <array>
#include <Eigen/Dense>
#include <vector>

namespace splexecutor
{
    namespace models
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

        template <size_t dof>
        class ManipulatorModel
        {

            protected:

                DHParameters<dof> dh_parameters;

            public:

                ManipulatorModel(const DHParameters<dof>& dh_parameters) : dh_parameters(dh_parameters)
                {}
                virtual ~ManipulatorModel() = default;

                virtual std::vector<double> getActualQ() const = 0;
                virtual std::vector<double> getActualQd() const = 0;

                virtual void speedJ(const std::vector<double>& joint_velocities, const double& acceleration) = 0;
        };

    }
}