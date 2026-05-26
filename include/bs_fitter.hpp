/*
    Shubh Khandelwal
*/

#pragma once
#include "spline_definitions.hpp"
#include <vector>

class BSFitter
{

    private:

        Eigen::Matrix4d t_pos, t_vel, t_acc, t_jrk;
        size_t frequency, resolution;
        
        std::vector<Point> interpolate(const Point& p0, const Point& p1, const Point& p2, const Point& p3)
        {
        
            Eigen::Matrix4Xd P;
            P.row(0) = p0;
            P.row(1) = p1;
            P.row(2) = p2;
            P.row(3) = p3;
        
        }
    
    public:

        BSFitter(const size_t& frequency, const size_t& resolution) : frequency(frequency), resolution(resolution)
        {

            Eigen::Matrix4d basis_matrix;
            basis_matrix.row(0) <<  1/6, 2/3,  1/6,   0;
            basis_matrix.row(1) << -1/2,   0,  1/2,   0;
            basis_matrix.row(2) <<  1/2,  -1,  1/2,   0;
            basis_matrix.row(3) << -1/6, 1/2, -1/2, 1/6;

            Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(this->resolution, 0.0, 1.0);
            Eigen::MatrixX4d t_pos, t_vel, t_acc, t_jrk;

        }

        void fitSpline(const std::vector<Point>& path) const
        {
        }

        void evaluate(const double& total_time)
        {
            size_t num_steps = static_cast<size_t>(total_time * this->frequency);
        }
    
};

// class SplineGenerator:

//     def __init__(self, basis_matrix: np.array, resolution: int, epsilon: float):
//         self.basis_matrix = basis_matrix
//         self.resolution = resolution
//         self.epsilon = epsilon

//     def _interpolate(self, p0, p1, p2, p3):
//         a = self.basis_matrix @ np.array([p0, p1, p2, p3])
//         t = np.linspace(0, 1, self.resolution)
//         pos = np.array([np.ones_like(t), t, t ** 2, t ** 3]).T @ a
//         vel = np.array([np.zeros_like(t), np.ones_like(t), 2 * t, 3 * t ** 2]).T @ a
//         acc = np.array([np.zeros_like(t), np.zeros_like(t), 2 * np.ones_like(t), 6 * t]).T @ a
//         jrk = np.array([np.zeros_like(t), np.zeros_like(t), np.zeros_like(t), 6 * np.ones_like(t)]).T @ a
//         return pos, vel, acc, jrk
    
//     def get_path(self, waypoints: list):

//         clean_waypoints = [waypoints[0], waypoints[0]]
//         for i in range(1, len(waypoints)):
//             if np.linalg.norm(waypoints[i] - waypoints[i - 1]) > self.epsilon:
//                 clean_waypoints.append(waypoints[i])
//         clean_waypoints.append(clean_waypoints[-1])

//         P = []
//         V = []
//         A = []
//         J = []
//         for i in range(len(clean_waypoints) - 3):

//             p0 = clean_waypoints[i]
//             p1 = clean_waypoints[i + 1]
//             p2 = clean_waypoints[i + 2]
//             p3 = clean_waypoints[i + 3]
//             pos, vel, acc, jrk = self._interpolate(p0, p1, p2, p3)

//             P.append(pos)
//             V.append(vel)
//             A.append(acc)
//             J.append(jrk)

//         P = np.concatenate(P)
//         V = np.concatenate(V)
//         A = np.concatenate(A)
//         J = np.concatenate(J)
//         return P, V, A, J
    
// class BSplineGenerator(SplineGenerator):

//     def __init__(self, resolution, epsilon = 1e-6):
//         basis_matrix = 1 / 6 * np.array([
//             [1, 4, 1, 0],
//             [-3, 0, 3, 0],
//             [3, -6, 3, 0],
//             [-1, 3, -3, 1],
//         ])
//         super().__init__(basis_matrix, resolution = resolution, epsilon = epsilon)