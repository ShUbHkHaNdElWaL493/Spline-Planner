/*
    Shubh Khandelwal
*/

#include "spl/definitions.hpp"
#include "splplanner/planner.hpp"
#include <iostream>
#include <fstream>

int main()
{

    spl::VectorRepresentation p0(3), p1(3), p2(3), p3(3);
    p0 << 1, 2, 3;
    p1 << 4, 5, 6;
    p2 << 7, 8, 9;
    p3 << 10, 11, 12;

    splplanner::Planner planner(2.0, 1.0, 500);
    spl::Trajectory trajectory = planner.plan({p0, p1, p2, p3});

    std::ofstream file("points.txt");
    for (size_t i = 0; i < trajectory.pos.size(); i++)
    {
        file << trajectory.pos[i] << std::endl;
    }
    file.close();

    return 0;

}