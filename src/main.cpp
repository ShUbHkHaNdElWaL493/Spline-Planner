/*
    Shubh Khandelwal
*/

#include "spline_definitions.hpp"
#include "crs_generator.hpp"
#include "time_optimizer.hpp"
#include <iostream>
#include <fstream>

int main()
{

    splplanner::SplineVector p0(3), p1(3), p2(3), p3(3);
    p0 << 1, 2, 3;
    p1 << 4, 5, 6;
    p2 << 7, 8, 9;
    p3 << 10, 11, 12;

    splplanner::TimeOptimizer time_optimizer(2.0, 1.0);

    splplanner::CRSGenerator crs_generator(50);
    std::vector<splplanner::SplineVector> path = crs_generator.getPath({p0, p1, p2, p3});
    auto result1 = time_optimizer.getOptimalKineticParameters(path);
    std::cout << result1.first << " " << result1.second << std::endl;

    // BSFitter bs_fitter(500, 50);
    // SplineTrajectory trajectory = bs_fitter.fitSpline(path);
    // auto result2 = time_optimizer.getOptimalKineticParameters(trajectory.pos);
    // std::cout << result2.first << " " << result2.second << std::endl;

    // std::ofstream file("points.txt");
    // for (size_t i = 0; i < trajectory.pos.size(); i++)
    // {
    //     file << trajectory.pos[i] << std::endl;
    // }
    // file.close();

    return 0;

}