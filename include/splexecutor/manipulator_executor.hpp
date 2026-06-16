/*
    Shubh Khandelwal
*/

#pragma once
#include "executor.hpp"
#include <memory>
#include "models/manipulator_model.hpp"

namespace splexecutor
{
    class ManipulatorExecutor : public Executor
    {

        private:

            size_t num_dims;
            std::unique_ptr<models::ManipulatorModel> manipulator_model;

            Eigen::MatrixXd getJacobian(const std::vector<double>& q) const
            {

                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                Eigen::MatrixXd J = Eigen::MatrixXd::Zero(this->num_dims, dh_parameters.dof);

                std::vector<Eigen::Matrix4d> T_matrices(dh_parameters.dof + 1);
                T_matrices[0] = Eigen::Matrix4d::Identity();

                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (size_t i = 0; i < dh_parameters.dof; ++i)
                {
                    T = T * dh_parameters.transform(q[i], i);
                    T_matrices[i + 1] = T;
                }

                Eigen::Vector3d o_n = T_matrices[dh_parameters.dof].block<3, 1>(0, 3);
                for (size_t i = 0; i < dh_parameters.dof; ++i)
                {
                    Eigen::Vector3d z_i_minus_1 = T_matrices[i].block<3, 1>(0, 2);
                    Eigen::Vector3d o_i_minus_1 = T_matrices[i].block<3, 1>(0, 3);
                    Eigen::Vector3d J_v = z_i_minus_1.cross(o_n - o_i_minus_1);
                    for (size_t j = 0; j < this->num_dims; ++j)
                    {
                        J(j, i) = J_v(j);
                    }
                }

                return J;

            }

            std::vector<double> getQd(const std::vector<double>& q, const spl::VectorRepresentation& x_dot) const
            {

                size_t dof = q.size();
                double lambda = 0.05;
                Eigen::MatrixXd J = this->getJacobian(q);
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(this->num_dims, this->num_dims);
                Eigen::MatrixXd J_T = J.transpose();

                Eigen::VectorXd x_dot_eigen = x_dot.transpose();
                Eigen::VectorXd q_dot_eigen = J_T * (J * J_T + (lambda * lambda) * I).inverse() * x_dot_eigen;
                std::vector<double> joint_velocities(dof);
                for (size_t i = 0; i < dof; ++i)
                {
                    joint_velocities[i] = q_dot_eigen(i);
                }
                
                return joint_velocities;

            }

            void execute(const spl::VectorRepresentation& out) override
            {
                std::vector<double> q = this->manipulator_model->getActualQ();
                std::vector<double> qd = this->getQd(q, out);
                this->manipulator_model->speedJ(qd);
            }

        public:

            ManipulatorExecutor(size_t frequency, size_t num_dims, std::unique_ptr<models::ManipulatorModel>& manipulator_model) :
            Executor(frequency),
            num_dims(num_dims),
            manipulator_model(std::move(manipulator_model))
            {}

            ~ManipulatorExecutor() override
            {
                this->manipulator_model.reset();
            }

            std::vector<spl::VectorRepresentation> getJointPositions(const std::vector<double>& q) const
            {

                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                std::vector<spl::VectorRepresentation> joint_positions(dh_parameters.dof + 1, spl::VectorRepresentation::Zero(this->num_dims));

                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (size_t i = 0; i < dh_parameters.dof; ++i)
                {
                    T = T * dh_parameters.transform(q[i], i);
                    for (size_t j = 0; j < this->num_dims; ++j)
                    {
                        joint_positions[i + 1](j) = T(j, 3);
                    }
                }

                return joint_positions;

            }

            void executeTrajectory(const spl::Trajectory& trajectory)
            {
                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                std::lock_guard<std::mutex> lock(this->state_output_mutex);
                for (size_t i = 0; i < trajectory.vel.size(); ++i)
                {
                    this->output.push(trajectory.vel[i]);
                }
            }

    };
}