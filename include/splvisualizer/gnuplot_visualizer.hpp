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

            FILE* gnuplot_pipe;

        public:

            GnuplotVisualizer(const spl::Trajectory& path) : Visualizer(path), gnuplot_pipe(popen("gnuplot", "w"))
            {

                if (!gnuplot_pipe)
                {
                    std::cerr << "<ERROR>: Failed to open gnuplot pipe." << std::endl;
                }

                size_t num_dims = this->trajectory[0].pos.cols();
                fprintf(gnuplot_pipe, "set terminal qt size 800, 1200\n");
                fprintf(gnuplot_pipe, "set multiplot layout %zu,1 title 'gnuplot Visualizer' font ',12'\n", num_dims);

                for (size_t i = 0; i < num_dims; ++i)
                {
                    fprintf(gnuplot_pipe, "set title 'Dimension %zu vs Time'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xlabel 'Time (s)'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Dimension %zu'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xrange [0.0:1.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-2.0:2.0]\n");
                    fprintf(gnuplot_pipe, "plot '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                    for (size_t j = 0; j < this->trajectory.size(); ++j)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", this->trajectory[j].u, this->trajectory[j].pos(i));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                }

                fprintf(gnuplot_pipe, "unset multiplot\n");
                fflush(gnuplot_pipe);

            }

            ~GnuplotVisualizer()
            {
                if (gnuplot_pipe)
                {
                    pclose(gnuplot_pipe);
                }
            }

            void visualize(const std::vector<spl::VectorRepresentation>& q) override
            {

                size_t num_dims = this->trajectory[0].pos.cols();
                fprintf(gnuplot_pipe, "set title 'gnuplot Visualizer' font ',12'\n");

                fprintf(gnuplot_pipe, "set xlabel 'X'\n");
                fprintf(gnuplot_pipe, "set ylabel 'Y'\n");
                fprintf(gnuplot_pipe, "set xrange [-2.0:2.0]\n");
                fprintf(gnuplot_pipe, "set yrange [-2.0:2.0]\n");

                if (num_dims == 2)
                {
                    fprintf(gnuplot_pipe, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                    fprintf(gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& joint_position : q)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", joint_position(0), joint_position(1));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                    for (const spl::TrajectoryPoint& trajectory_point : this->trajectory)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", trajectory_point.pos(0), trajectory_point.pos(1));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                } else if (num_dims == 3)
                {
                    fprintf(gnuplot_pipe, "set zlabel 'Z'\n");
                    fprintf(gnuplot_pipe, "set zrange [-2.0:2.0]\n");
                    fprintf(gnuplot_pipe, "splot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                    fprintf(gnuplot_pipe, "      '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& joint_position : q)
                    {
                        fprintf(gnuplot_pipe, "%f %f %f\n", joint_position(0), joint_position(1), joint_position(2));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                    for (const spl::TrajectoryPoint& trajectory_point : this->trajectory)
                    {
                        fprintf(gnuplot_pipe, "%f %f %f\n", trajectory_point.pos(0), trajectory_point.pos(1), trajectory_point.pos(2));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                }

                fflush(gnuplot_pipe);

            }

    };
}