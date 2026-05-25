/*
    Shubh Khandelwal
*/

#pragma once
#include "spline_definitions.hpp"
#include <vector>

class CRSGenerator
{

    private:

        size_t resolution;

        std::vector<Point> interpolate(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const
        {

            auto getT = [](const double& t, const Point& p0, const Point& p1)
            {
                double distance = (p1 - p0).norm();
                return t + std::max(sqrt(distance), 0.001);
            };

            double t0 = 0.0;
            double t1 = getT(t0, p0, p1);
            double t2 = getT(t1, p1, p2);
            double t3 = getT(t2, p2, p3);

            Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(this->resolution, t1, t2);
            
            auto safeDt = [](double dt)
            {
                return (std::abs(dt) < 1e-12) ? 1e-12 : dt;
            };

            Eigen::MatrixX3d A1 = ((t1 - t.array()) / safeDt(t1 - t0)).matrix() * p0 + ((t.array() - t0) / safeDt(t1 - t0)).matrix() * p1;
            Eigen::MatrixX3d A2 = ((t2 - t.array()) / safeDt(t2 - t1)).matrix() * p1 + ((t.array() - t1) / safeDt(t2 - t1)).matrix() * p2;
            Eigen::MatrixX3d A3 = ((t3 - t.array()) / safeDt(t3 - t2)).matrix() * p2 + ((t.array() - t2) / safeDt(t3 - t2)).matrix() * p3;

            Eigen::MatrixX3d B1 = A1.array().colwise() * ((t2 - t.array()) / safeDt(t2 - t0)).array() +
                                 A2.array().colwise() * ((t.array() - t0) / safeDt(t2 - t0)).array();
            Eigen::MatrixX3d B2 = A2.array().colwise() * ((t3 - t.array()) / safeDt(t3 - t1)).array() +
                                 A3.array().colwise() * ((t.array() - t1) / safeDt(t3 - t1)).array();

            Eigen::MatrixX3d C = B1.array().colwise() * ((t2 - t.array()) / safeDt(t2 - t1)).array() +
                                B2.array().colwise() * ((t.array() - t1) / safeDt(t2 - t1)).array();

            std::vector<Point> segment(C.rows());
            for (size_t i = 0; i < C.rows(); ++i)
            {
                segment[i] = C.row(i);
            }

            return segment;
        
        }
    
    public:

        CRSGenerator(size_t resolution) : resolution(resolution)
        {}

        std::vector<Point> getPath(const std::vector<Point>& waypoints) const
        {

            std::vector<Point> clean_waypoints;
            clean_waypoints.push_back(waypoints[0]);
            clean_waypoints.push_back(waypoints[0]);
            for (size_t i = 1; i < waypoints.size(); i++)
            {
                if ((waypoints[i] - waypoints[i - 1]).norm() > 1e-6)
                {
                    clean_waypoints.push_back(waypoints[i]);
                }
            }
            clean_waypoints.push_back(waypoints[waypoints.size() - 1]);

            std::vector<Point> path;
            path.push_back(clean_waypoints[0]);
            for (size_t i = 0; i < clean_waypoints.size() - 3; i++)
            {
                std::vector<Point> segment = this->interpolate(clean_waypoints[i], clean_waypoints[i + 1], clean_waypoints[i + 2], clean_waypoints[i + 3]);
                path.insert(path.end(), segment.begin() + 1, segment.end());
            }

            return path;

        }

};