/*
    Shubh Khandelwal
*/

#pragma once
#include <memory>
#include <mutex>
#include <queue>
#include "spl/definitions.hpp"
#include <thread>

namespace splexecutor
{
    class Executor
    {

        private:

        mutable std::mutex state_vpt_mutex;
        std::atomic<bool> is_running, is_spinning;
            std::chrono::time_point<std::chrono::steady_clock> spin_start_time;
            std::thread spin_thread;
            std::vector<std::pair<double, spl::VectorRepresentation>> visualizer_planned_trajectory;

            void spinLoop()
            {
                while (this->is_running)
                {
                    auto loop_start_time = std::chrono::steady_clock::now();
                    if (this->is_spinning)
                    {
                        {
                            std::lock_guard<std::mutex> lock(this->state_output_mutex);
                            if (!this->output.empty())
                            {
                                this->execute(this->output.front());
                                {
                                    std::lock_guard<std::mutex> lock(this->state_vpt_mutex);
                                    this->visualizer_planned_trajectory.push_back({this->getTime(), this->output.front().pos});
                                }
                                this->output.pop();
                            }
                        }
                    }
                    std::this_thread::sleep_until(loop_start_time + std::chrono::duration<double>(this->dt));
                }
            }
            
            virtual void execute(const spl::TrajectoryPoint& t) = 0;
            
        protected:

            double dt;
            mutable std::mutex state_output_mutex;
            std::queue<spl::TrajectoryPoint> output;

            double getTime()
            {
                std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - this->spin_start_time;
                return elapsed.count();
            }

        public:

            Executor(size_t frequency) :
            is_running(true),
            is_spinning(false),
            dt(1.0 / frequency)
            {
                this->spin_thread = std::thread(&Executor::spinLoop, this);
            }

            virtual ~Executor()
            {
                this->is_running = false;
                this->spin_thread.join();
            }

            std::vector<std::pair<double, spl::VectorRepresentation>> getVisualizerPlannedTrajectory()
            {
                std::lock_guard<std::mutex> lock(this->state_vpt_mutex);
                return this->visualizer_planned_trajectory;
            }

            void spin()
            {
                this->is_spinning = true;
                this->spin_start_time = std::chrono::steady_clock::now();
            }

    };
}