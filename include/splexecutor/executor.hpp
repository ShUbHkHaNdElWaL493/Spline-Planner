/*
    Shubh Khandelwal
*/

#pragma once
#include <memory>
#include <mutex>
#include <queue>
#include "spl/definitions.hpp"
#include "visualizers/visualizer.hpp"
#include <thread>

namespace splexecutor
{
    class Executor
    {

        private:

            std::atomic<bool> is_running, is_spinning;
            std::chrono::time_point<std::chrono::steady_clock> spin_start_time;
            std::thread spin_thread;
            std::thread visualize_thread;

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
                                std::lock_guard<std::mutex> lock(this->state_visualizer_mutex);
                                if (this->visualizer)
                                {
                                    this->visualizer->append(this->getTime(), this->output.front().pos);
                                }
                                this->output.pop();
                            }
                        }
                    }
                    std::this_thread::sleep_until(loop_start_time + std::chrono::duration<double>(this->dt));
                }
            }

            void visualizeLoop()
            {
                while (this->is_running)
                {
                    auto loop_start_time = std::chrono::steady_clock::now();
                    if (this->is_spinning)
                    {
                        this->visualize();
                    }
                    std::this_thread::sleep_until(loop_start_time + std::chrono::milliseconds(30));
                }
            }
            
            virtual void execute(const spl::TrajectoryPoint& t) = 0;
            virtual void visualize() = 0;
            
        protected:

            double dt;
            mutable std::mutex state_output_mutex, state_visualizer_mutex;
            std::queue<spl::TrajectoryPoint> output;
            std::unique_ptr<visualizers::Visualizer> visualizer;

            double getTime()
            {
                std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - this->spin_start_time;
                return elapsed.count();
            }

        public:

            Executor(size_t frequency, std::unique_ptr<visualizers::Visualizer> visualizer = nullptr) :
            is_running(true),
            is_spinning(false),
            dt(1.0 / frequency),
            visualizer(std::move(visualizer))
            {
                this->spin_thread = std::thread(&Executor::spinLoop, this);
                if (this->visualizer)
                {
                    this->visualize_thread = std::thread(&Executor::visualizeLoop, this);
                }
            }

            virtual ~Executor()
            {
                this->is_running = false;
                if (this->visualize_thread.joinable())
                {
                    this->visualize_thread.join();
                }
                this->spin_thread.join();
                this->visualizer.reset();
            }

            void spin()
            {
                this->is_spinning = true;
                this->spin_start_time = std::chrono::steady_clock::now();
            }

    };
}