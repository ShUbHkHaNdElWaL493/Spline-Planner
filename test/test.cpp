/*
    Shubh Khandelwal
*/

#define NUM_DIMS 3
#define MAX_VEL_PLANNER    1.0
#define MAX_ACC_PLANNER    0.05
#define MAX_ACC_EXECUTOR  40.0
#define MODEL_UR
#define VISUALIZER_GMV

#include "spl/definitions.hpp"
#include "splexecutor/manipulator_executor.hpp"
#include "splexecutor/models/simulated_manipulator_model.hpp"
#include "splexecutor/models/ur_manipulator_model.hpp"
#include "splplanner/planner.hpp"
#include "splvisualizer/gnuplot_visualizer.hpp"

int main(int argc, char **argv)
{

    std::string ur_model = std::getenv("UR_MODEL");
    std::string ur_series = std::getenv("UR_SERIES");

    if (ur_model == "" || ur_series == "")
    {
        throw std::runtime_error("Environment variables not set. Check .env file.");
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
        throw std::runtime_error("Invalid UR series.");
    }
    
    std::unique_ptr<splexecutor::models::ManipulatorModel> model;

    #ifdef MODEL_UR
        // UR Manipulator Model
        model = std::make_unique<splexecutor::models::URManipulatorModel>(
            "resources/dh_parameters/" + ur_model + ".csv",
            "192.168.56.101"
        );
    #elifdef MODEL_SIM
        // Simulated Manipulator Model
        model = std::make_unique<splexecutor::models::SimulatedManipulatorModel>(
            "resources/dh_parameters/" + ur_model + ".csv",
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            frequency
        );
    #endif

    splexecutor::ManipulatorExecutor executor(frequency, NUM_DIMS, model, MAX_ACC_EXECUTOR);
    executor.spin();

    splplanner::Planner planner(MAX_VEL_PLANNER, MAX_ACC_PLANNER, frequency);
    std::vector<spl::VectorRepresentation> p(5, spl::VectorRepresentation(NUM_DIMS));
    p[0] = executor.getInitialQ();
    p[1] <<  0.4,  0.0, 0.3;
    p[2] <<  0.0,  0.4, 0.3;
    p[3] << -0.4,  0.0, 0.3;
    p[4] <<  0.0, -0.4, 0.3;
    spl::Trajectory trajectory = planner.plan(p);

    std::unique_ptr<splvisualizer::Visualizer> visualizer;
    #ifdef VISUALIZER_GDV
        // Gnuplot Dimensional Visualizer
        visualizer = std::make_unique<splvisualizer::GnuplotDimensionalVisualizer>(NUM_DIMS);
    #elifdef VISUALIZER_GMV
        // Gnuplot Manipulator Visualizer
        visualizer = std::make_unique<splvisualizer::GnuplotManipulatorVisualizer>(NUM_DIMS);
    #endif

    executor.executeTrajectory(trajectory);

    while (true)
    {
        auto loop_start_time = std::chrono::steady_clock::now();
        visualizer->visualize(executor.getVisualizerPlannedTrajectory(), executor.getVisualizationParameters());
        std::this_thread::sleep_until(loop_start_time + std::chrono::milliseconds(30));
    }

    return 0;

}