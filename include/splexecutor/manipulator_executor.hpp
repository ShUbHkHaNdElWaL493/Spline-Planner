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

            double max_acc;
            Eigen::MatrixXd J_initial;
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

            std::pair<std::vector<double>, double> getSpeedJParameters(
                std::vector<double>& q,
                std::vector<double>& qd,
                const spl::VectorRepresentation& x,
                const spl::VectorRepresentation& xd,
                const spl::VectorRepresentation& xdd,
                const double &kp
            )
            {

                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                size_t dof = q.size();
                Eigen::Map<Eigen::VectorXd> q_eigen(q.data(), q.size());
                Eigen::Map<Eigen::VectorXd> qd_eigen(qd.data(), qd.size());
                Eigen::VectorXd x_eigen = x.transpose();
                Eigen::VectorXd xd_eigen = xd.transpose();
                Eigen::VectorXd xdd_eigen = xdd.transpose();

                double lambda = EPSILON;
                Eigen::MatrixXd J = this->getJacobian(q);
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(this->num_dims, this->num_dims);
                Eigen::MatrixXd J_T = J.transpose();
                Eigen::MatrixXd J_I = J_T * (J * J_T + (lambda * lambda) * I).inverse();
                Eigen::MatrixXd J_d = (J - this->J_initial) / dt;
                this->J_initial = J;

                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (size_t i = 0; i < dof; ++i)
                {
                    T = T * dh_parameters.transform(q[i], i);
                }
                Eigen::VectorXd x_actual_eigen(this->num_dims);
                for (size_t i = 0; i < this->num_dims; ++i)
                {
                    x_actual_eigen(i) = T(i, 3);
                }
                Eigen::VectorXd q_dot_eigen = J_I * (xd_eigen + kp * (x_eigen - x_actual_eigen));
                std::vector<double> q_dot(dof);
                for (size_t i = 0; i < dof; ++i)
                {
                    q_dot[i] = q_dot_eigen(i);
                }

                Eigen::VectorXd q_dot_dot_eigen = J_I * (xdd_eigen - J_d * qd_eigen + kp * (xd_eigen - J * qd_eigen));
                double q_dot_dot = q_dot_dot_eigen.cwiseAbs().maxCoeff();
                q_dot_dot = std::min(q_dot_dot, this->max_acc);
                
                return {q_dot, q_dot_dot};

            }

            void execute(const spl::VectorRepresentation& out, const spl::VectorRepresentation& out_d, const spl::VectorRepresentation& out_dd) override
            {
                std::vector<double> q = this->manipulator_model->getActualQ();
                std::vector<double> qd = this->manipulator_model->getActualQd();
                std::pair<std::vector<double>, double> speedj_parameters = this->getSpeedJParameters(q, qd, out, out_d, out_dd, 0.05);
                this->manipulator_model->speedJ(speedj_parameters.first, speedj_parameters.second);
                this->output.pop();
                if (this->output.empty())
                {
                    this->manipulator_model->speedJ(std::vector<double>(q.size(), 0.0), this->max_acc);
                }
            }

        public:

            ManipulatorExecutor(size_t frequency, size_t num_dims, std::unique_ptr<models::ManipulatorModel>& manipulator_model, const double& max_acc) :
            Executor(frequency),
            max_acc(max_acc),
            num_dims(num_dims),
            manipulator_model(std::move(manipulator_model))
            {}

            ~ManipulatorExecutor() override
            {
                this->manipulator_model.reset();
            }

            spl::VectorRepresentation getInitialQ()
            {
                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                std::vector<double> q = this->manipulator_model->getActualQ();
                this->J_initial = this->getJacobian(q);
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (size_t i = 0; i < dh_parameters.dof; ++i)
                {
                    T = T * dh_parameters.transform(q[i], i);
                }
                return T.block<3, 1>(0, 3).transpose();
            }

            std::vector<spl::VectorRepresentation> getJointPositions()
            {

                const models::DHParameters dh_parameters = this->manipulator_model->getDHParameters();
                std::vector<double> q = this->manipulator_model->getActualQ();
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
                for (size_t i = 0; i < trajectory.size(); ++i)
                {
                    this->output.push(trajectory[i]);
                }
            }

    };
}