/*
    Shubh Khandelwal
*/

#pragma once
#include "spl/definitions.hpp"

namespace splplanner
{
    class CRSFitter
    {

        private:

            size_t resolution;

            static double getT(const double t, const spl::VectorRepresentation& p0, const spl::VectorRepresentation& p1)
            {
                double distance = (p1 - p0).norm();
                return t + std::max(sqrt(distance), 0.001);
            }

            static double safeDt(const double dt)
            {
                return (std::abs(dt) < EPSILON) ? EPSILON : dt;
            }

            std::vector<spl::VectorRepresentation> interpolate(
                const spl::VectorRepresentation& p0,
                const spl::VectorRepresentation& p1,
                const spl::VectorRepresentation& p2,
                const spl::VectorRepresentation& p3
            ) const
            {

                double t0 = 0.0;
                double t1 = getT(t0, p0, p1);
                double t2 = getT(t1, p1, p2);
                double t3 = getT(t2, p2, p3);

                Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(this->resolution, t1, t2);

                Eigen::MatrixXd A1 = ((t1 - t.array()) / safeDt(t1 - t0)).matrix() * p0 + ((t.array() - t0) / safeDt(t1 - t0)).matrix() * p1;
                Eigen::MatrixXd A2 = ((t2 - t.array()) / safeDt(t2 - t1)).matrix() * p1 + ((t.array() - t1) / safeDt(t2 - t1)).matrix() * p2;
                Eigen::MatrixXd A3 = ((t3 - t.array()) / safeDt(t3 - t2)).matrix() * p2 + ((t.array() - t2) / safeDt(t3 - t2)).matrix() * p3;

                Eigen::MatrixXd B1 = A1.array().colwise() * ((t2 - t.array()) / safeDt(t2 - t0)).array() +
                                    A2.array().colwise() * ((t.array() - t0) / safeDt(t2 - t0)).array();
                Eigen::MatrixXd B2 = A2.array().colwise() * ((t3 - t.array()) / safeDt(t3 - t1)).array() +
                                    A3.array().colwise() * ((t.array() - t1) / safeDt(t3 - t1)).array();

                Eigen::MatrixXd C = B1.array().colwise() * ((t2 - t.array()) / safeDt(t2 - t1)).array() +
                                    B2.array().colwise() * ((t.array() - t1) / safeDt(t2 - t1)).array();

                std::vector<spl::VectorRepresentation> segment(C.rows());
                for (size_t i = 0; i < C.rows(); ++i)
                {
                    segment[i] = C.row(i);
                }

                return segment;

            }

        public:

            CRSFitter(const size_t resolution) : resolution(resolution)
            {}

            std::vector<spl::VectorRepresentation> fitSpline(const std::vector<spl::VectorRepresentation>& waypoints) const
            {

                std::vector<spl::VectorRepresentation> clean_waypoints;
                clean_waypoints.push_back(waypoints[0]);
                for (size_t i = 1; i < waypoints.size(); ++i)
                {
                    if ((waypoints[i] - waypoints[i - 1]).norm() > EPSILON)
                    {
                        clean_waypoints.push_back(waypoints[i]);
                    }
                }

                std::vector<spl::VectorRepresentation> points;
                points.push_back(clean_waypoints[0]);
                points.insert(points.end(), clean_waypoints.begin(), clean_waypoints.end());
                points.push_back(clean_waypoints.back());

                std::vector<spl::VectorRepresentation> path;
                for (size_t i = 0; i < points.size() - 3; ++i)
                {
                    std::vector<spl::VectorRepresentation> segment = this->interpolate(points[i], points[i + 1], points[i + 2], points[i + 3]);
                    if (i == points.size() - 4)
                    {
                        path.insert(path.end(), segment.begin(), segment.end());
                    } else
                    {
                        path.insert(path.end(), segment.begin(), segment.end() - 1);
                    }
                }

                return path;

            }

    };
}