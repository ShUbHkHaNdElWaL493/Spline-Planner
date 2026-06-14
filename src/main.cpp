/*
    Shubh Khandelwal
*/

#include "spl/definitions.hpp"
#include "splplanner/planner.hpp"
#include "splvisualizer/gnuplot_visualizer.hpp"
#include <iostream>
#include <fstream>

int main()
{

    spl::VectorRepresentation p0(3), p1(3), p2(3), p3(3);
    p0 <<  12.4,  -8.1,   3.5;
    p1 << -14.2,   6.7,  -2.3;
    p2 <<   0.5,  11.9, -13.0;
    p3 <<  -5.8, -10.4,  14.1;

    splplanner::Planner planner(2.0, 1.0, 500);
    std::pair<std::vector<double>, spl::Trajectory> path = planner.plan({p0, p1, p2, p3});
    splvisualizer::GnuplotVisualizer visualizer(path);

    std::ofstream file("points.txt");
    for (size_t i = 0; i < path.second.pos.size(); i++)
    {
        file << path.second.pos[i] << std::endl;
    }
    file.close();

    return 0;

}