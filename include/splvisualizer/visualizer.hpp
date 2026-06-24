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

            spl::Trajectory trajectory;

        public:

            Visualizer(const spl::Trajectory& path) : trajectory(path)
            {}

            virtual ~Visualizer() = default;
            virtual void visualize(const std::vector<spl::VectorRepresentation>& q) = 0;

    };
}