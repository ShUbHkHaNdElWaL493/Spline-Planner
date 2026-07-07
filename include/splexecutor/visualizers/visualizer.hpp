/*
    Shubh Khandelwal
*/

#pragma once
#include "spl/definitions.hpp"

namespace splexecutor
{
    namespace visualizers
    {
        class Visualizer
        {

            protected:
                std::vector<std::pair<double, spl::VectorRepresentation>> planned_trajectory, real_trajectory;

            public:

                void append(const double& t, const spl::VectorRepresentation& pos)
                {
                    this->planned_trajectory.push_back({t, pos});
                }
                
                virtual ~Visualizer() = default;
                virtual void visualize(const double& u, const std::vector<spl::VectorRepresentation>& q) = 0;

        };
    }
}