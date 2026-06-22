/*
    Shubh Khandelwal
*/

#pragma once
#include "manipulator_model.hpp"
#include <mutex>
#include <thread>

namespace splexecutor
{
    namespace models
    {
        class SimulatedManipulatorModel : public ManipulatorModel
        {

            private:

                
                double dt;
                mutable std::mutex state_mutex;
                std::atomic<bool> is_running;
                std::thread sim_thread;
                std::vector<double> q, qd;

                void loop()
                {
                    while (is_running)
                    {
                        auto start_time = std::chrono::steady_clock::now();
                        {
                            std::lock_guard<std::mutex> lock(state_mutex);
                            for (size_t i = 0; i < this->dh_parameters.dof; ++i)
                            {
                                this->q[i] += this->qd[i] * dt;
                            }
                        }
                        std::this_thread::sleep_until(start_time + std::chrono::duration<double>(dt));
                    }
                }

            public:

                SimulatedManipulatorModel(const DHParameters& dh_parameters, const std::vector<double>& initial_q, const size_t& frequency) :
                ManipulatorModel(dh_parameters),
                is_running(true),
                dt(1.0 / frequency),
                q(initial_q),
                qd(this->dh_parameters.dof, 0.0)
                { sim_thread = std::thread(&SimulatedManipulatorModel::loop, this); }

                SimulatedManipulatorModel(const std::string& dh_parameters_file, const std::vector<double>& initial_q, const size_t& frequency) :
                ManipulatorModel(dh_parameters_file),
                is_running(true),
                dt(1.0 / frequency),
                q(initial_q),
                qd(this->dh_parameters.dof, 0.0)
                { sim_thread = std::thread(&SimulatedManipulatorModel::loop, this); }

                ~SimulatedManipulatorModel() override
                {
                    is_running = false;
                    sim_thread.join();
                }

                std::vector<double> getActualQ() override
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    return q;
                }

                void speedJ(const std::vector<double>& qd) override
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    this->qd = qd;
                }

        };
    }
}