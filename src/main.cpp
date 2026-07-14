/*
    Shubh Khandelwal
*/

#define NUM_DIMS 3
#define MAX_VEL_PLANNER    1.0
#define MAX_ACC_PLANNER    0.05
#define MAX_ACC_EXECUTOR  40.0
#define MODEL_UR
#define VISUALIZER_GMV

#include <csignal>
#include "spl/definitions.hpp"
#include "splexecutor/manipulator_executor.hpp"
#include "splexecutor/models/simulated_manipulator_model.hpp"
#include "splexecutor/models/ur_manipulator_model.hpp"
#include "splplanner/planner.hpp"
#include "splvisualizer/gnuplot_visualizer.hpp"

std::atomic<bool> loop = true;
void stopLoop(int signum) { loop = false; }

int main()
{

    std::string ur_model = std::getenv("UR_MODEL");
    std::string ur_series = std::getenv("UR_SERIES");

    if (ur_model == "" || ur_series == "")
    {
        throw std::runtime_error("[ERROR] Environment variables not set. Check .env file.");
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
        throw std::runtime_error("[ERROR] Invalid UR series.");
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
    std::vector<spl::VectorRepresentation> p(9, spl::VectorRepresentation(NUM_DIMS));
    p[0] = executor.getInitialQ();
    p[1] <<  0.4,  0.0, 0.3;
    p[2] <<  0.0,  0.4, 0.3;
    p[3] << -0.4,  0.0, 0.3;
    p[4] <<  0.0, -0.4, 0.3;
    p[5] <<  0.0, -0.4, 0.4;
    p[6] << -0.4,  0.0, 0.4;
    p[7] <<  0.0,  0.4, 0.4;
    p[8] <<  0.4,  0.0, 0.4;
    spl::Trajectory trajectory = planner.plan(p);

    #ifdef VISUALIZER_GDV
        // Gnuplot Dimensional Visualizer
        std::unique_ptr<splvisualizer::GnuplotDimensionalVisualizer> visualizer = std::make_unique<splvisualizer::GnuplotDimensionalVisualizer>(NUM_DIMS);
    #elifdef VISUALIZER_GMV
        // Gnuplot Manipulator Visualizer
        std::unique_ptr<splvisualizer::GnuplotManipulatorVisualizer> visualizer = std::make_unique<splvisualizer::GnuplotManipulatorVisualizer>(NUM_DIMS);
    #endif

    executor.executeTrajectory(trajectory);
    std::signal(SIGINT, stopLoop);

    #ifndef NDEBUG
    std::pair<spl::Log, spl::Log> executor_logs;
    #endif

    while (loop)
    {
        auto loop_start_time = std::chrono::steady_clock::now();
        #ifndef NDEBUG
        executor_logs = executor.getLog();
        #ifdef VISUALIZER_GDV
        visualizer->visualize(executor_logs.first, executor_logs.second);
        #elifdef VISUALIZER_GMV
        visualizer->visualize(executor_logs.first, executor_logs.second, executor.getJointPositions());
        #endif
        #else
        #ifdef VISUALIZER_GMV
        visualizer->visualize(executor.getJointPositions());
        #endif
        #endif
        std::this_thread::sleep_until(loop_start_time + std::chrono::milliseconds(30));
    }

    #ifndef NDEBUG

    std::ofstream planning_log_file("log/planning_log.csv");
    planning_log_file << "time,pos,vel,acc,jrk" << std::endl;
    for (const spl::LogEntry log_entry : executor_logs.first)
    {
        planning_log_file << log_entry.first << ",";
        planning_log_file << "[";
        planning_log_file << log_entry.second.pos(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            planning_log_file << ";" << log_entry.second.pos(i);
        }
        planning_log_file << "],";
        planning_log_file << "[";
        planning_log_file << log_entry.second.vel(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            planning_log_file << ";" << log_entry.second.vel(i);
        }
        planning_log_file << "],";
        planning_log_file << "[";
        planning_log_file << log_entry.second.acc(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            planning_log_file << ";" << log_entry.second.acc(i);
        }
        planning_log_file << "],";
        planning_log_file << "[";
        planning_log_file << log_entry.second.jrk(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            planning_log_file << ";" << log_entry.second.jrk(i);
        }
        planning_log_file << "]" << std::endl;
    }
    planning_log_file.close();

    std::ofstream execution_log_file("log/execution_log.csv");
    execution_log_file << "time,pos,vel,acc,jrk" << std::endl;
    for (const spl::LogEntry log_entry : executor_logs.second)
    {
        execution_log_file << log_entry.first << ",";
        execution_log_file << "[";
        execution_log_file << log_entry.second.pos(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            execution_log_file << ";" << log_entry.second.pos(i);
        }
        execution_log_file << "],";
        execution_log_file << "[";
        execution_log_file << log_entry.second.vel(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            execution_log_file << ";" << log_entry.second.vel(i);
        }
        execution_log_file << "],";
        execution_log_file << "[";
        execution_log_file << log_entry.second.acc(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            execution_log_file << ";" << log_entry.second.acc(i);
        }
        execution_log_file << "],";
        execution_log_file << "[";
        execution_log_file << log_entry.second.jrk(0);
        for (size_t i = 1; i < NUM_DIMS; i++)
        {
            execution_log_file << ";" << log_entry.second.jrk(i);
        }
        execution_log_file << "]" << std::endl;
    }
    execution_log_file.close();

    #endif

    return 0;

}