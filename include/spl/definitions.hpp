/*
    Shubh Khandelwal
*/

#pragma once
#include <Eigen/Dense>
#include <vector>

#define EPSILON 1e-6

namespace spl
{

    typedef Eigen::RowVectorXd VectorRepresentation;

    /**
     * @brief Struct holding the values of the position, velocity, acceleration and jerk.
     * 
     * @tparam pos Vector holding positions of the subject
     * @tparam vel Vector holding velocities of the subject
     * @tparam acc Vector holding accelerations of the subject
     * @tparam jrk Vector holding jerks of the subject
     */
    struct Trajectory
    {
        std::vector<VectorRepresentation> pos, vel, acc, jrk;
    };

}