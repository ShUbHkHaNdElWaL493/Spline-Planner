/*
    Shubh Khandelwal
*/

#pragma once
#include <Eigen/Dense>
#include <vector>

#define EPSILON 1e-6

namespace splplanner
{

    typedef Eigen::RowVectorXd SplineVector;

    struct SplineTrajectory
    {
        std::vector<SplineVector> pos, vel, acc, jrk;
    };

}