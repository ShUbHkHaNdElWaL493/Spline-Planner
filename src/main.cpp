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
#include "splexecutor/models/ur_manipulator_model.hpp"
#include "splexecutor/manipulator_executor.hpp"
#include <stdexcept>

int main()
{

    const std::string manipulator_model = "sim";
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

    splplanner::Planner planner(0.1, 30, frequency);
    std::unique_ptr<splexecutor::models::ManipulatorModel> model;

    if (manipulator_model == "ur")
    {
        // UR Manipulator Model
        model = std::make_unique<splexecutor::models::URManipulatorModel>(
            "resources/dh_parameters/" + ur_model + ".csv",
            "192.168.56.101"
        );
    } else if (manipulator_model == "sim")
    {
        // Simulated Manipulator Model
        model = std::make_unique<splexecutor::models::SimulatedManipulatorModel>(
            "resources/dh_parameters/" + ur_model + ".csv",
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            frequency
        );
    }

    splexecutor::ManipulatorExecutor executor(frequency, NUM_DIMS, model);
    executor.spin();

    std::vector<spl::VectorRepresentation> p(4, spl::VectorRepresentation(NUM_DIMS));
    p[0] = executor.getInitialQ();
    p[1] <<  0.4,  0.0,  0.3;
    p[2] <<  0.1, 0.45, 0.15;
    p[3] << 0.35, -0.3,  0.5;
    std::pair<std::vector<double>, spl::Trajectory> path = planner.plan(p);

    std::ofstream file("points.txt");
    for (size_t i = 0; i < path.second.pos.size(); i++)
    {
        file << path.second.pos[i] << std::endl;
    }
    file.close();

    splvisualizer::GnuplotVisualizer visualizer(path);
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    executor.executeTrajectory(path.second);

    while (true)
    {
        visualizer.visualize(executor.getJointPositions());
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return 0;

}