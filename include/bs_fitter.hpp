/*
    Shubh Khandelwal
*/

#pragma once
#include "spline_definitions.hpp"

class BSFitter
{

    private:

        double max_vel, max_acc;
        Eigen::MatrixX4d coef_pos, coef_vel, coef_acc, coef_jrk;
        size_t frequency, resolution;
        
        SplineTrajectory interpolate(const SplineVector& p0, const SplineVector& p1, const SplineVector& p2, const SplineVector& p3) const
        {
        
            Eigen::Matrix4Xd P(4, p0.cols());
            P.row(0) = p0;
            P.row(1) = p1;
            P.row(2) = p2;
            P.row(3) = p3;

            Eigen::MatrixXd pos = coef_pos * P;
            Eigen::MatrixXd vel = coef_vel * P;
            Eigen::MatrixXd acc = coef_acc * P;
            Eigen::MatrixXd jrk = coef_jrk * P;

            size_t vector_size = pos.rows();
            std::vector<SplineVector> pos_vector(vector_size), vel_vector(vector_size), acc_vector(vector_size), jrk_vector(vector_size);
            for (size_t i = 0; i < vector_size; ++i)
            {
                pos_vector[i] = pos.row(i);
                vel_vector[i] = vel.row(i);
                acc_vector[i] = acc.row(i);
                jrk_vector[i] = jrk.row(i);
            }

            return {pos_vector, vel_vector, acc_vector, jrk_vector};
        
        }

        std::pair<double, double> getOptimalKineticParameters(const std::vector<SplineVector>& path) const
        {

            double length_dense = 0.0;
            double length_sparse = 0.0;

            for (size_t i = 1; i < path.size(); ++i)
            {
                length_dense += (path[i] - path[i - 1]).norm();
                if (i % 2 == 0) 
                {
                    length_sparse += (path[i] - path[i - 2]).norm();
                }
            }

            double distance =  (4.0 * length_dense - length_sparse) / 3.0;
            double time = (distance / this->max_vel) + (this->max_vel / this->max_acc);

            return {distance, time};
        
        }
    
    public:

        BSFitter(const size_t& frequency, const size_t& resolution, const double& max_vel, const double& max_acc) :
        max_vel(max_vel),
        max_acc(max_acc),
        coef_pos(resolution, 4),
        coef_vel(resolution, 4),
        coef_acc(resolution, 4),
        coef_jrk(resolution, 4),
        frequency(frequency),
        resolution(resolution)
        {

            Eigen::Matrix4d basis_matrix;
            basis_matrix.row(0) <<  1.0/6.0,  2.0/3.0,  1.0/6.0,      0.0;
            basis_matrix.row(1) << -1.0/2.0,      0.0,  1.0/2.0,      0.0;
            basis_matrix.row(2) <<  1.0/2.0,     -1.0,  1.0/2.0,      0.0;
            basis_matrix.row(3) << -1.0/6.0,  1.0/2.0, -1.0/2.0,  1.0/6.0;

            Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(resolution, 0.0, 1.0);
            Eigen::VectorXd t2 = t.array().square(); 
            Eigen::VectorXd t3 = t.array().cube();

            Eigen::MatrixX4d t_pos(resolution, 4);
            t_pos.col(0).setOnes();
            t_pos.col(1) = t;
            t_pos.col(2) = t2;
            t_pos.col(3) = t3;

            Eigen::MatrixX4d t_vel(resolution, 4);
            t_vel.col(0).setZero();
            t_vel.col(1).setOnes();
            t_vel.col(2) = 2.0 * t;
            t_vel.col(3) = 3.0 * t2;

            Eigen::MatrixX4d t_acc(resolution, 4);
            t_acc.col(0).setZero();
            t_acc.col(1).setZero();
            t_acc.col(2).setConstant(2.0);
            t_acc.col(3) = 6.0 * t;

            Eigen::MatrixX4d t_jrk(resolution, 4); 
            t_jrk.col(0).setZero();
            t_jrk.col(1).setZero();
            t_jrk.col(2).setZero();
            t_jrk.col(3).setConstant(6.0);
            
            this->coef_pos = t_pos * basis_matrix;
            this->coef_vel = t_vel * basis_matrix;
            this->coef_acc = t_acc * basis_matrix;
            this->coef_jrk = t_jrk * basis_matrix;

        }

        SplineTrajectory fitSpline(const std::vector<SplineVector>& waypoints) const
        {

            std::vector<SplineVector> clean_waypoints;
            clean_waypoints.push_back(waypoints[0]);
            clean_waypoints.push_back(waypoints[0]);
            for (size_t i = 1; i < waypoints.size(); i++)
            {
                if ((waypoints[i] - waypoints[i - 1]).norm() > EPSILON)
                {
                    clean_waypoints.push_back(waypoints[i]);
                }
            }
            clean_waypoints.push_back(waypoints[waypoints.size() - 1]);

            SplineTrajectory trajectory;
            for (size_t i = 0; i < clean_waypoints.size() - 3; i++)
            {
                SplineTrajectory segment = this->interpolate(clean_waypoints[i], clean_waypoints[i + 1], clean_waypoints[i + 2], clean_waypoints[i + 3]);
                if (i == clean_waypoints.size() - 4)
                {
                    trajectory.pos.insert(trajectory.pos.end(), segment.pos.begin(), segment.pos.end());
                    trajectory.vel.insert(trajectory.vel.end(), segment.vel.begin(), segment.vel.end());
                    trajectory.acc.insert(trajectory.acc.end(), segment.acc.begin(), segment.acc.end());
                    trajectory.jrk.insert(trajectory.jrk.end(), segment.jrk.begin(), segment.jrk.end());
                } else
                {
                    trajectory.pos.insert(trajectory.pos.end(), segment.pos.begin(), segment.pos.end() - 1);
                    trajectory.vel.insert(trajectory.vel.end(), segment.vel.begin(), segment.vel.end() - 1);
                    trajectory.acc.insert(trajectory.acc.end(), segment.acc.begin(), segment.acc.end() - 1);
                    trajectory.jrk.insert(trajectory.jrk.end(), segment.jrk.begin(), segment.jrk.end() - 1);
                }
            }

            return trajectory;

        }

        void evaluate(const std::vector<SplineVector>& waypoints) const
        {

            SplineTrajectory trajectory = this->fitSpline(waypoints);
            std::pair<double, double> kinetic_parameters = this->getOptimalKineticParameters(trajectory.pos);
            double total_distance = kinetic_parameters.first, total_time = kinetic_parameters.second;

            size_t num_steps = static_cast<size_t>(total_time * this->frequency);

        }
    
};