/*
    Shubh Khandelwal
*/

#pragma once
#include <Eigen/Dense>
#include <vector>

#define EPSILON 1e-6

namespace spl
{

    typedef Eigen::VectorXd VectorRepresentation;
    
    /**
     * @brief Struct holding the values of the position, velocity, acceleration and jerk.
     * 
     * @tparam pos Position of the object
     * @tparam vel Velocity of the object
     * @tparam acc Acceleration of the object
     * @tparam jrk Jerk of the object
     */
    struct TrajectoryPoint { VectorRepresentation pos, vel, acc, jrk; };
    typedef std::vector<TrajectoryPoint> Trajectory;

    typedef std::pair<double, spl::TrajectoryPoint> LogEntry;
    typedef std::vector<LogEntry> Log;

}