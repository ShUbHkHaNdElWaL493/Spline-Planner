/*
    Shubh Khandelwal
*/

#pragma once
#include "manipulator_model.hpp"
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

namespace splexecutor
{
    namespace models
    {
        class URManipulatorModel : public ManipulatorModel
        {

            private:

                ur_rtde::DashboardClient rtde_dc;
                std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_ci;
                std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_ri;

            public:

                URManipulatorModel(const DHParameters& dh_parameters, const std::string& robot_ip) :
                ManipulatorModel(dh_parameters),
                rtde_dc(robot_ip)
                {

                    this->rtde_dc.connect();
                    this->rtde_dc.powerOn();
                    this->rtde_dc.brakeRelease();
                    std::this_thread::sleep_for(std::chrono::seconds(3));

                    this->rtde_ri = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip);
                    this->rtde_ci = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip);
                    
                }

                URManipulatorModel(const std::string& dh_parameters_file, const std::string& robot_ip) :
                ManipulatorModel(dh_parameters_file),
                rtde_dc(robot_ip)
                {

                    this->rtde_dc.connect();
                    this->rtde_dc.powerOn();
                    this->rtde_dc.brakeRelease();
                    std::this_thread::sleep_for(std::chrono::seconds(3));

                    this->rtde_ri = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip);
                    this->rtde_ci = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip);

                }

                ~URManipulatorModel() override
                {
                    this->rtde_ci.reset();
                    this->rtde_ri.reset();
                    this->rtde_dc.powerOff();
                    this->rtde_dc.disconnect();
                }

                std::vector<double> getActualQ() override
                {
                    return this->rtde_ri->getActualQ();
                }

                void speedJ(const std::vector<double>& qd) override
                {
                    this->rtde_ci->speedJ(qd);
                }

        };
    }
}