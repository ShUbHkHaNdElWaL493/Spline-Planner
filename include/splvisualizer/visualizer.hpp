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

            std::vector<double> u;
            spl::Trajectory trajectory;

        public:

            Visualizer(const std::pair<std::vector<double>, spl::Trajectory>& path) :
            u(path.first),
            trajectory(path.second)
            {}

            virtual ~Visualizer() = default;
            virtual void visualize(const std::vector<spl::VectorRepresentation>& q) = 0;

    };
}