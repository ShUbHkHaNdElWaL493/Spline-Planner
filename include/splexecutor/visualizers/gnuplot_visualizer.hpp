/*
    Shubh Khandelwal
*/

#pragma once
#include <stdexcept>
#include "visualizer.hpp"

namespace splexecutor
{
    namespace visualizers
    {

        class GnuplotDimensionalVisualizer : public Visualizer
        {

            private:
                FILE* gnuplot_pipe;
                size_t num_dims;

            public:

                GnuplotDimensionalVisualizer(const size_t& num_dims) : gnuplot_pipe(popen("gnuplot", "w")), num_dims(num_dims)
                {

                    if (!(this->gnuplot_pipe))
                    {
                        throw std::runtime_error("[ERROR] Failed to open gnuplot_pipe.");
                    }

                }

                ~GnuplotDimensionalVisualizer()
                {
                    if (this->gnuplot_pipe)
                    {
                        pclose(this->gnuplot_pipe);
                    }
                }

                void visualize(const double& u, const std::vector<spl::VectorRepresentation>& joint_positions) override
                {
                    this->real_trajectory.push_back({u, joint_positions.back()});
                    fprintf(this->gnuplot_pipe, "set terminal qt size 800, 1200\n");
                    fprintf(this->gnuplot_pipe, "set multiplot layout %zu,1 title 'Dimensional Plots' font ',12'\n", num_dims);
                    for (size_t i = 0; i < num_dims; ++i)
                    {
                        fprintf(this->gnuplot_pipe, "set title 'Dimension %zu vs Time'\n", i + 1);
                        fprintf(this->gnuplot_pipe, "set xlabel 'Time (s)'\n");
                        fprintf(this->gnuplot_pipe, "set ylabel 'Dimension %zu'\n", i + 1);
                        fprintf(this->gnuplot_pipe, "set xrange [0.0:1.0]\n");
                        fprintf(this->gnuplot_pipe, "set yrange [-2.0:2.0]\n");
                        fprintf(this->gnuplot_pipe, "plot '-' with lines lw 2 lc rgb 'green' title 'Planned D%zu(t)', \\\n", i + 1);
                        fprintf(this->gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'red' title 'Real D%zu(t)'\n", i + 1);
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->planned_trajectory)
                        {
                            fprintf(
                                this->gnuplot_pipe,
                                "%f %f\n",
                                trajectory_point.first,
                                trajectory_point.second(i)
                            );
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->real_trajectory)
                        {
                            fprintf(
                                this->gnuplot_pipe,
                                "%f %f\n",
                                trajectory_point.first,
                                trajectory_point.second(i)
                            );
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                    }
                    fprintf(this->gnuplot_pipe, "unset multiplot\n");
                    fflush(this->gnuplot_pipe);
                }

        };

        class GnuplotManipulatorVisualizer : public Visualizer
        {

            private:
                FILE* gnuplot_pipe;
                size_t num_dims;

            public:

                GnuplotManipulatorVisualizer(const size_t& num_dims) : gnuplot_pipe(popen("gnuplot", "w")), num_dims(num_dims)
                {

                    if (num_dims != 2 && num_dims != 3)
                    {
                        throw std::runtime_error("[ERROR] GnuplotManipulatorVisualizer can be used only for 2-D or 3-D models.");
                    }

                    if (!(this->gnuplot_pipe))
                    {
                        throw std::runtime_error("[ERROR] Failed to open gnuplot_pipe.");
                    }

                }

                ~GnuplotManipulatorVisualizer()
                {
                    if (this->gnuplot_pipe)
                    {
                        pclose(this->gnuplot_pipe);
                    }
                }

                void visualize(const double& u, const std::vector<spl::VectorRepresentation>& joint_positions) override
                {
                    this->real_trajectory.push_back({u, joint_positions.back()});
                    fprintf(this->gnuplot_pipe, "set title 'Trajectory Visualizer' font ',12'\n");
                    fprintf(this->gnuplot_pipe, "set xlabel 'X'\n");
                    fprintf(this->gnuplot_pipe, "set ylabel 'Y'\n");
                    fprintf(this->gnuplot_pipe, "set xrange [-2.0:2.0]\n");
                    fprintf(this->gnuplot_pipe, "set yrange [-2.0:2.0]\n");
                    if (num_dims == 2)
                    {
                        fprintf(this->gnuplot_pipe, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(this->gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'green' title 'Planned Trajectory', \\\n");
                        fprintf(this->gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'red' title 'Real Trajectory'\n");
                        for (const spl::VectorRepresentation& joint_position : joint_positions)
                        {
                            fprintf(this->gnuplot_pipe, "%f %f\n", joint_position(0), joint_position(1));
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->planned_trajectory)
                        {
                            fprintf(this->gnuplot_pipe, "%f %f\n", trajectory_point.second(0), trajectory_point.second(1));
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->real_trajectory)
                        {
                            fprintf(this->gnuplot_pipe, "%f %f\n", trajectory_point.second(0), trajectory_point.second(1));
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                    } else if (num_dims == 3)
                    {
                        fprintf(this->gnuplot_pipe, "set zlabel 'Z'\n");
                        fprintf(this->gnuplot_pipe, "set zrange [-2.0:2.0]\n");
                        fprintf(this->gnuplot_pipe, "splot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(this->gnuplot_pipe, "      '-' with lines lw 2 lc rgb 'green' title 'Planned Trajectory', \\\n");
                        fprintf(this->gnuplot_pipe, "      '-' with lines lw 2 lc rgb 'red' title 'Real Trajectory'\n");
                        for (const spl::VectorRepresentation& joint_position : joint_positions)
                        {
                            fprintf(this->gnuplot_pipe, "%f %f %f\n", joint_position(0), joint_position(1), joint_position(2));
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->planned_trajectory)
                        {
                            fprintf(
                                this->gnuplot_pipe,
                                "%f %f %f\n",
                                trajectory_point.second(0),
                                trajectory_point.second(1),
                                trajectory_point.second(2)
                            );
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                        for (const std::pair<double, spl::VectorRepresentation>& trajectory_point : this->real_trajectory)
                        {
                            fprintf(
                                this->gnuplot_pipe,
                                "%f %f %f\n",
                                trajectory_point.second(0),
                                trajectory_point.second(1),
                                trajectory_point.second(2)
                            );
                        }
                        fprintf(this->gnuplot_pipe, "e\n");
                    }
                    fflush(this->gnuplot_pipe);
                }

        };

    }
}