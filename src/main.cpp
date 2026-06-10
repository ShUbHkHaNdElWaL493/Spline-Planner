/*
    Shubh Khandelwal
*/

#include <iostream>
#include "../include/spline_definitions.hpp"
#include "../include/crs_fitter.hpp"
#include "../include/bs_fitter.hpp"
#include "../include/kinematics_optimizer.hpp"
#include <fstream>

int main()
{

    splplanner::VectorRepresentation p0(3), p1(3), p2(3), p3(3);
    p0 << 1, 2, 3;
    p1 << 4, 5, 6;
    p2 << 7, 8, 9;
    p3 << 10, 11, 12;

    splplanner::CRSFitter crs_fitter(50);
    splplanner::BSFitter bs_fitter(500, 0.005);
    splplanner::KinematicsOptimizer kinematics_optimizer(2.0, 1.0);

    std::vector<splplanner::VectorRepresentation> path = crs_fitter.fitSpline({p0, p1, p2, p3});
    std::vector<splplanner::Spline> splines = bs_fitter.fitSpline(path);
    auto kinematics_params = kinematics_optimizer.getOptimalKinematicsParameters(splines);
    std::cout << kinematics_params.first << " " << kinematics_params.second << std::endl;
    splplanner::Trajectory trajectory = bs_fitter.evaluate(splines, kinematics_params.second);

    std::ofstream file("points.txt");
    for (size_t i = 0; i < trajectory.pos.size(); i++)
    {
        file << trajectory.pos[i] << std::endl;
    }
    file.close();

    return 0;

}