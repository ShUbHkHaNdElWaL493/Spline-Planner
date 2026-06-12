/*
    Shubh Khandelwal
*/

#pragma once
#include "spl/definitions.hpp"
#include "crs_fitter.hpp"
#include "bs_fitter.hpp"
#include "kinematics_optimizer.hpp"

namespace splplanner
{
    class Planner
    {

        private:

            struct LogEntry
            {
                double distance, time;
                std::vector<double> u;
                spl::Trajectory trajectory;
            };

            CRSFitter crs_fitter;
            BSFitter bs_fitter;
            KinematicsOptimizer kinematics_optimizer;
            std::vector<LogEntry> log;

        public:

            Planner(const double& max_vel, const double& max_acc, const size_t& frequency, const size_t& resolution = 50, const double& s = 0.005) :
            crs_fitter(resolution),
            bs_fitter(frequency, s),
            kinematics_optimizer(max_vel, max_acc)
            {}

            spl::Trajectory plan(const std::vector<spl::VectorRepresentation>& waypoints)
            {
                std::vector<spl::VectorRepresentation> path = this->crs_fitter.fitSpline(waypoints);
                std::vector<Spline> splines = this->bs_fitter.fitSpline(path);
                std::pair<double, double> kinematics_params = this->kinematics_optimizer.getOptimalKinematicsParameters(splines);
                std::pair<std::vector<double>, spl::Trajectory> result = bs_fitter.evaluate(splines, kinematics_params.second);
                log.push_back({
                    kinematics_params.first,
                    kinematics_params.second,
                    result.first,
                    result.second.pos,
                    result.second.vel,
                    result.second.acc,
                    result.second.jrk
                });
                return result.second;
            }

    };
}