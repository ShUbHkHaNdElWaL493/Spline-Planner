/*
    Shubh Khandelwal
*/

#pragma once
#include <mutex>
#include <queue>
#include "spl/definitions.hpp"
#include <thread>

namespace splexecutor
{
    class Executor
    {

        private:

            std::atomic<bool> is_running, is_spinning;
            std::thread spin_thread;

            void loop()
            {
                while (is_running)
                {
                    auto start_time = std::chrono::steady_clock::now();
                    if (is_spinning)
                    {
                        {
                            std::lock_guard<std::mutex> lock(this->state_output_mutex);
                            if (!this->output.empty())
                            {
                                this->execute(this->output.front().pos, this->output.front().vel, this->output.front().acc);
                            }
                        }
                    }
                    std::this_thread::sleep_until(start_time + std::chrono::duration<double>(this->dt));
                }
            }
            
            virtual void execute(const spl::VectorRepresentation& out, const spl::VectorRepresentation& out_d, const spl::VectorRepresentation& out_dd) = 0;
            
        protected:

            double dt;
            mutable std::mutex state_output_mutex;
            std::queue<spl::TrajectoryPoint> output;

        public:

            Executor(size_t frequency) : is_running(true), is_spinning(false), dt(1.0 / frequency)
            { spin_thread = std::thread(&Executor::loop, this); }

            virtual ~Executor()
            {
                is_running = false;
                spin_thread.join();
            }

            void spin() { is_spinning = true; }

    };
}