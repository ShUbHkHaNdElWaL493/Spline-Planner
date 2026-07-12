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

            #ifndef NDEBUG
            spl::Log log_planning, log_execution;
            mutable std::mutex state_log_mutex;
            #endif
            std::atomic<bool> is_running, is_spinning;
            std::chrono::time_point<std::chrono::steady_clock> spin_start_time;
            std::thread spin_thread;

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
                                #ifndef NDEBUG
                                {
                                    std::lock_guard<std::mutex> lock(this->state_log_mutex);
                                    double time = this->getTime();
                                    this->log_planning.push_back({time, this->output.front()});
                                    this->log_execution.push_back({time, this->getTrajectoryPoint()});
                                }
                                #endif
                                this->execute(this->output.front());
                            }
                        }
                    }
                    std::this_thread::sleep_until(loop_start_time + std::chrono::duration<double>(this->dt));
                }
            }
            
            virtual void execute(const spl::TrajectoryPoint& t) = 0;

            #ifndef NDEBUG
            virtual spl::TrajectoryPoint getTrajectoryPoint() = 0;
            #endif
            
        protected:

            double dt;
            mutable std::mutex state_output_mutex;
            std::queue<spl::TrajectoryPoint> output;

            double getTime() const
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

            #ifndef NDEBUG
            std::pair<spl::Log, spl::Log> getLog()
            {
                std::lock_guard<std::mutex> lock(this->state_log_mutex);
                return {this->log_planning, this->log_execution};
            }
            #endif

            void spin()
            {
                this->is_spinning = true;
                this->spin_start_time = std::chrono::steady_clock::now();
            }

    };
}