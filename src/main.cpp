/*
    Shubh Khandelwal
*/

#define NUM_DIMS 3

#include <cstdlib>
#include <iostream>
#include <fstream>
#include "spl/definitions.hpp"
#include "splplanner/planner.hpp"
#include "splvisualizer/gnuplot_visualizer.hpp"
#include "splexecutor/models/simulated_manipulator_model.hpp"
#include "splexecutor/manipulator_executor.hpp"
#include <stdexcept>

int main()
{

    std::string ur_model = std::getenv("UR_MODEL");
    std::string ur_series = std::getenv("UR_SERIES");

    if (ur_model == "" || ur_series == "")
    {
        std::runtime_error("Environment variables not set. Check .env file.");
    }

    spl::VectorRepresentation p0(NUM_DIMS), p1(NUM_DIMS), p2(NUM_DIMS), p3(NUM_DIMS);
    p0 <<  12.4,  -8.1,   3.5;
    p1 << -14.2,   6.7,  -2.3;
    p2 <<   0.5,  11.9, -13.0;
    p3 <<  -5.8, -10.4,  14.1;

    splplanner::Planner planner(2.0, 1.0, 500);
    std::pair<std::vector<double>, spl::Trajectory> path = planner.plan({p0, p1, p2, p3});

    std::ofstream file("points.txt");
    for (size_t i = 0; i < path.second.pos.size(); i++)
    {
        file << path.second.pos[i] << std::endl;
    }
    file.close();

    splvisualizer::GnuplotVisualizer visualizer(path);

    size_t frequency;
    if (ur_series == "cb3")
    {
        frequency = 125;
    } else if (ur_series == "e-series")
    {
        frequency = 500;
    } else
    {
        std::runtime_error("Invalid UR series.");
    }

    std::unique_ptr<splexecutor::models::ManipulatorModel> model = std::make_unique<splexecutor::models::SimulatedManipulatorModel>(
        "resources/dh_parameters/" + ur_model + ".csv",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        frequency
    );

    splexecutor::ManipulatorExecutor executor(frequency, NUM_DIMS, model);
    executor.spin();
    executor.executeTrajectory(path.second);

    while (true)
    {}

    return 0;

}