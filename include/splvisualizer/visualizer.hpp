/*
    Shubh Khandelwal
*/

#pragma once
#include "spl/definitions.hpp"

namespace splvisualizer
{
    class Visualizer
    {
        public:
            virtual ~Visualizer() = default;
            virtual void visualize(const spl::Log& log_planning, const spl::Log& log_execution) = 0;
    };
}