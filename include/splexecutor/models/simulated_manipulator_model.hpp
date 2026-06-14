/*
    Shubh Khandelwal
*/

#pragma once
#include <atomic>
#include <chrono>
#include "manipulator_model.hpp"
#include <mutex>
#include <thread>

namespace splexecutor
{
    namespace models
    {
        template <size_t dof>
        class SimulatedManipulatorModel : public ManipulatorModel<dof>
        {

            private:

                mutable std::mutex state_mutex;
                std::chrono::duration<double> dt;
                std::thread sim_thread;
                std::vector<double> q, qd;

                void loop()
                {
                    auto start_time = std::chrono::steady_clock::now();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        for (size_t i = 0; i < dof; ++i)
                        {
                            q[i] += qd[i] * dt;
                        }
                    }
                    std::this_thread::sleep_until(start_time + sleep_duration);
                }

            public:

                SimulatedManipulatorModel(const DHParameters<dof>& dh_parameters, const std::vector<double>& initial_q, const size_t& frequency) :
                ManipulatorModel<dof>(dh_parameters),
                dt(std::chrono::duration<double>(1.0 / frequency)),
                q(initial_q),
                qd(dof, 0.0)
                { sim_thread = std::thread(&SimulatedManipulatorModel::loop, this); }

                ~SimulatedManipulatorModel() override
                {
                    if (sim_thread.joinable()) { sim_thread.join(); }
                }

                std::vector<double> getActualQ() const override
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    return q;
                }

                std::vector<double> getActualQd() const override
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    return qd;
                }

                void speedJ(const std::vector<double>& joint_velocities, const double& acceleration) override
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    qd = joint_velocities;
                }

        };
    }
}