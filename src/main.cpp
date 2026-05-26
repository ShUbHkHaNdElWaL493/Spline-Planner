/*
    Shubh Khandelwal
*/

#include "crs_generator.hpp"
#include <iostream>

int main()
{

    Point p0, p1, p2, p3;
    p0 << 1, 2, 3;
    p1 << 4, 5, 6;
    p2 << 7, 8, 9;
    p3 << 10, 11, 12;

    CRSGenerator crs_generator(50, 2, 1);
    std::vector<Point> path = crs_generator.getPath({p0, p1, p2, p3});
    std::pair<double, double> result = crs_generator.getOptimalTime(path);

    for (size_t i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << std::endl;
    }

    std::cout << result.first << " " << result.second << std::endl;

    return 0;

}