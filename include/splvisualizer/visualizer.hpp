/*
    Shubh Khandelwal
*/

#pragma once
#include "spl/definitions.hpp"

namespace splvisualizer
{
    class Visualizer
    {

        protected:
            std::vector<std::pair<double, spl::VectorRepresentation>> real_trajectory;

        public:
            virtual ~Visualizer() = default;
            virtual void visualize(
                const std::vector<std::pair<double, spl::VectorRepresentation>>& planned_trajectory,
                const std::pair<double, std::vector<spl::VectorRepresentation>>& timed_joint_positions
            ) = 0;

    };
}