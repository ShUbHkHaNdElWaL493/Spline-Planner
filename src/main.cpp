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

    spl::VectorRepresentation p0(NUM_DIMS), p1(NUM_DIMS), p2(NUM_DIMS), p3(NUM_DIMS);
    p0 << -0.8172, -0.2329, 0.0628;
    p1 <<     0.1,     0.1,    0.2;
    p2 <<     0.2,     0.1,    0.2;
    p3 <<    -0.2,    -0.2,    0.2;

    splplanner::Planner planner(2.0, 1.0, frequency);
    std::pair<std::vector<double>, spl::Trajectory> path = planner.plan({p0, p1, p2, p3});

    std::ofstream file("points.txt");
    for (size_t i = 0; i < path.second.pos.size(); i++)
    {
        file << path.second.pos[i] << std::endl;
    }
    file.close();

    splvisualizer::GnuplotVisualizer visualizer(path);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::unique_ptr<splexecutor::models::ManipulatorModel> model = std::make_unique<splexecutor::models::SimulatedManipulatorModel>(
        "resources/dh_parameters/" + ur_model + ".csv",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        frequency
    );

    splexecutor::ManipulatorExecutor executor(frequency, NUM_DIMS, model);
    executor.spin();
    executor.executeTrajectory(path.second);

    std::cout << std::endl;
    while (true)
    {
        visualizer.visualize(executor.getJointPositions());
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return 0;

}