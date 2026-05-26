/*
    Shubh Khandelwal
*/

#include "crs_generator.hpp"
#include "bs_fitter.hpp"
#include <iostream>

int main()
{

    SplineVector p0(3), p1(3), p2(3), p3(3);
    p0 << 1, 2, 3;
    p1 << 4, 5, 6;
    p2 << 7, 8, 9;
    p3 << 10, 11, 12;

    CRSGenerator crs_generator(50);
    std::vector<SplineVector> path = crs_generator.getPath({p0, p1, p2, p3});

    BSFitter bs_fitter(500, 50, 2.0, 1.0);
    SplineTrajectory trajectory = bs_fitter.fitSpline(path);

    for (size_t i = 0; i < trajectory.pos.size(); i++)
    {
        std::cout << trajectory.pos[i] << std::endl;
    }

    return 0;

}