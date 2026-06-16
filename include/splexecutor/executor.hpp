/*
    Shubh Khandelwal
*/

#pragma once
#include <atomic>
#include <mutex>
#include <queue>
#include "spl/definitions.hpp"
#include <thread>

namespace splexecutor
{
    class Executor
    {

        private:

            double dt;
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
                                this->execute(this->output.front());
                                this->output.pop();
                            }
                        }
                    }
                    std::this_thread::sleep_until(start_time + std::chrono::duration<double>(this->dt));
                }
            }
            
            virtual void execute(const spl::VectorRepresentation& out) = 0;
            
        protected:

            mutable std::mutex state_output_mutex;
            std::queue<spl::VectorRepresentation> output;

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