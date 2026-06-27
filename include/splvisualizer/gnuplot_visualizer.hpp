/*
    Shubh Khandelwal
*/

#pragma once
#include <iostream>
#include "visualizer.hpp"

namespace splvisualizer
{
    class GnuplotVisualizer : Visualizer
    {

        private:

            FILE* pipe_multiplot;
            FILE* pipe_visualizer;

        public:

            GnuplotVisualizer(const spl::Trajectory& path) : Visualizer(path), pipe_multiplot(popen("gnuplot", "w"))
            {

                if (!pipe_multiplot)
                {
                    std::cerr << "<ERROR>: Failed to open pipe_multiplot." << std::endl;
                }

                size_t num_dims = this->trajectory[0].pos.cols();

                fprintf(pipe_multiplot, "set terminal qt size 800, 1200\n");
                fprintf(pipe_multiplot, "set multiplot layout %zu,1 title 'Dimensional Plots' font ',12'\n", num_dims);
                for (size_t i = 0; i < num_dims; ++i)
                {
                    fprintf(pipe_multiplot, "set title 'Dimension %zu vs Time'\n", i + 1);
                    fprintf(pipe_multiplot, "set xlabel 'Time (s)'\n");
                    fprintf(pipe_multiplot, "set ylabel 'Dimension %zu'\n", i + 1);
                    fprintf(pipe_multiplot, "set xrange [0.0:1.0]\n");
                    fprintf(pipe_multiplot, "set yrange [-2.0:2.0]\n");
                    fprintf(pipe_multiplot, "plot '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                    for (size_t j = 0; j < this->trajectory.size(); ++j)
                    {
                        fprintf(pipe_multiplot, "%f %f\n", this->trajectory[j].u, this->trajectory[j].pos(i));
                    }
                    fprintf(pipe_multiplot, "e\n");
                }
                fprintf(pipe_multiplot, "unset multiplot\n");
                fflush(pipe_multiplot);

                if (num_dims == 2 || num_dims == 3)
                {
                    pipe_visualizer = popen("gnuplot", "w");
                    if (!pipe_visualizer)
                    {
                        std::cerr << "<ERROR>: Failed to open pipe_visualizer." << std::endl;
                    }
                }

            }

            ~GnuplotVisualizer()
            {
                if (pipe_visualizer)
                {
                    pclose(pipe_visualizer);
                }
                if (pipe_multiplot)
                {
                    pclose(pipe_multiplot);
                }
            }

            void visualize(const std::vector<spl::VectorRepresentation>& q) override
            {
                if (pipe_visualizer)
                {
                    size_t num_dims = this->trajectory[0].pos.cols();
                    fprintf(pipe_visualizer, "set title 'Trajectory Visualizer' font ',12'\n");
                    fprintf(pipe_visualizer, "set xlabel 'X'\n");
                    fprintf(pipe_visualizer, "set ylabel 'Y'\n");
                    fprintf(pipe_visualizer, "set xrange [-2.0:2.0]\n");
                    fprintf(pipe_visualizer, "set yrange [-2.0:2.0]\n");
                    if (num_dims == 2)
                    {
                        fprintf(pipe_visualizer, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(pipe_visualizer, "     '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                        for (const spl::VectorRepresentation& joint_position : q)
                        {
                            fprintf(pipe_visualizer, "%f %f\n", joint_position(0), joint_position(1));
                        }
                        fprintf(pipe_visualizer, "e\n");
                        for (const spl::TrajectoryPoint& trajectory_point : this->trajectory)
                        {
                            fprintf(pipe_visualizer, "%f %f\n", trajectory_point.pos(0), trajectory_point.pos(1));
                        }
                        fprintf(pipe_visualizer, "e\n");
                    } else if (num_dims == 3)
                    {
                        fprintf(pipe_visualizer, "set zlabel 'Z'\n");
                        fprintf(pipe_visualizer, "set zrange [-2.0:2.0]\n");
                        fprintf(pipe_visualizer, "splot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(pipe_visualizer, "      '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                        for (const spl::VectorRepresentation& joint_position : q)
                        {
                            fprintf(pipe_visualizer, "%f %f %f\n", joint_position(0), joint_position(1), joint_position(2));
                        }
                        fprintf(pipe_visualizer, "e\n");
                        for (const spl::TrajectoryPoint& trajectory_point : this->trajectory)
                        {
                            fprintf(pipe_visualizer, "%f %f %f\n", trajectory_point.pos(0), trajectory_point.pos(1), trajectory_point.pos(2));
                        }
                        fprintf(pipe_visualizer, "e\n");
                    }
                    fflush(pipe_visualizer);
                }
            }

    };
}