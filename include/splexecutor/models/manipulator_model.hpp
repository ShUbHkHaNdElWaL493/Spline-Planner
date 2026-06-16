/*
    Shubh Khandelwal
*/

#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace splexecutor
{
    namespace models
    {

        struct DHParameters
        {

            size_t dof;
            std::vector<double> d;
            std::vector<double> a;
            std::vector<double> alpha;

            DHParameters(size_t dof) : dof(dof), d(dof, 0.0), a(dof, 0.0), alpha(dof, 0.0)
            {}

            DHParameters(size_t dof, const std::vector<double>& d, const std::vector<double>& a, const std::vector<double>& alpha) :
            dof(dof),
            d(d),
            a(a),
            alpha(alpha)
            {}

            Eigen::Matrix4d transform(const double& theta, const size_t& i) const
            {
                Eigen::Matrix4d T;
                if (i >= dof || i < 0)
                {
                    T = Eigen::Matrix4d::Identity();
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

        class ManipulatorModel
        {

            private:

                DHParameters extractDHParameters(const std::string& dh_parameters_file) const
                {

                    std::ifstream file(dh_parameters_file, std::ios::in);
                    if (!file.is_open())
                    {
                        std::runtime_error("Failed to open DH parameters file: " + dh_parameters_file + ".");
                    }

                    size_t dof = 0;
                    std::string line;
                    std::vector<std::vector<std::string>> parsedCsv;
                    while (std::getline(file, line))
                    {

                        std::stringstream lineStream(line);
                        std::string cell;
                        std::vector<std::string> parsedRow;

                        while (std::getline(lineStream, cell, ','))
                        {
                            parsedRow.push_back(cell);
                        }
                        parsedCsv.push_back(parsedRow);
                        dof++;

                    }
                    dof--;
                    file.close();

                    std::vector<double> d(dof, 0.0), a(dof, 0.0), alpha(dof, 0.0);
                    for (size_t i = 0; i < dof; ++i)
                    {
                        d[i] = std::stod(parsedCsv[i + 1][0]);
                        a[i] = std::stod(parsedCsv[i + 1][1]);
                        alpha[i] = std::stod(parsedCsv[i + 1][2]);
                    }

                    return {dof, d, a, alpha};

                }

            protected:

                DHParameters dh_parameters;

            public:

                ManipulatorModel(const DHParameters& dh_parameters) : dh_parameters(dh_parameters)
                {}

                ManipulatorModel(const std::string& dh_parameters_file) : dh_parameters(extractDHParameters(dh_parameters_file))
                {}

                const DHParameters& getDHParameters() const { return this->dh_parameters; }

                virtual ~ManipulatorModel() = default;
                virtual std::vector<double> getActualQ() const = 0;
                virtual void speedJ(const std::vector<double>& joint_velocities) = 0;
        };

    }
}