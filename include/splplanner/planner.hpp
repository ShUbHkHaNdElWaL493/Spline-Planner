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

            CRSFitter crs_fitter;
            BSFitter bs_fitter;
            KinematicsOptimizer kinematics_optimizer;
            #ifndef NDEBUG
            spl::Log log;
            #endif

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
                #ifndef NDEBUG
                for (size_t i = 0; i < result.first.size(); ++i)
                {
                    this->log.push_back({result.first[i], result.second[i]});
                }
                #endif
                return result.second;
            }

            #ifndef NDEBUG
            spl::Log getLog()
            {
                return this->log;
            }
            #endif

    };
}