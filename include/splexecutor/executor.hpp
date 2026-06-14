/*
    Shubh Khandelwal
*/

#pragma once
#include "../spl/definitions.hpp"

namespace splexecutor
{

    class Executor
    {

        private:

            double dt;

        public:

            Executor(size_t frequency) : dt(1.0 / frequency)
            {}

    };

};