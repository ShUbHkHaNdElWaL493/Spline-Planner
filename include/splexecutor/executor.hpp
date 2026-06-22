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
                            if (!this->output_d.empty())
                            {
                                size_t num_dims = this->output_d.front().cols();
                                this->execute(this->output_d.front(), this->output_dd.front());
                                this->output_d.pop();
                                this->output_dd.pop();
                                if (this->output_d.empty())
                                {
                                    this->execute(spl::VectorRepresentation::Zero(num_dims), spl::VectorRepresentation::Zero(num_dims));
                                }
                            }
                        }
                    }
                    std::this_thread::sleep_until(start_time + std::chrono::duration<double>(this->dt));
                }
            }
            
            virtual void execute(const spl::VectorRepresentation& out_d, const spl::VectorRepresentation& out_dd) = 0;
            
        protected:

            double dt;
            mutable std::mutex state_output_mutex;
            std::queue<spl::VectorRepresentation> output_d;
            std::queue<spl::VectorRepresentation> output_dd;

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