/*
    Shubh Khandelwal
*/

#include "spline_definitions.hpp"

class TimeOptimizer
{

    private:

        double max_vel, max_acc;
    
    public:

        TimeOptimizer(const double& max_vel, const double& max_acc) : max_vel(max_vel), max_acc(max_acc)
        {}

        std::pair<double, double> getOptimalKineticParameters(const std::vector<SplineVector>& path) const
        {

            double length_dense = 0.0;
            double length_sparse = 0.0;

            for (size_t i = 1; i < path.size(); ++i)
            {
                length_dense += (path[i] - path[i - 1]).norm();
                if (i % 2 == 0) 
                {
                    length_sparse += (path[i] - path[i - 2]).norm();
                }
            }

            double distance =  (4.0 * length_dense - length_sparse) / 3.0;
            double time = (distance / this->max_vel) + (this->max_vel / this->max_acc);

            return {distance, time};
        
        }
};