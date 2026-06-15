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

            GnuplotVisualizer(const std::pair<std::vector<double>, spl::Trajectory>& path) :
            Visualizer(path),
            gnuplot_pipe(popen("gnuplot -persist", "w"))
            {

                if (!gnuplot_pipe)
                {
                    std::cerr << "<ERROR>: Failed to open gnuplot pipe." << std::endl;
                }

                size_t num_dims = this->trajectory.pos[0].cols();
                fprintf(gnuplot_pipe, "set terminal qt size 800, 1200\n");
                fprintf(gnuplot_pipe, "clear\n");
                fprintf(gnuplot_pipe, "set multiplot layout %zu,1 title 'gnuplot Visualizer' font ',12'\n", (num_dims == 2 || num_dims == 3) ? num_dims + 1 : num_dims);

                for (size_t i = 0; i < num_dims; ++i)
                {
                    fprintf(gnuplot_pipe, "set title 'Dimension %zu vs Time'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xlabel 'Time (s)'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Dimension %zu'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "plot '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                    for (size_t j = 0; j < this->u.size(); ++j)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", this->u[j], this->trajectory.pos[j](i));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                }

                if (num_dims == 2)
                {
                    fprintf(gnuplot_pipe, "set title 'Trajectory'\n");
                    fprintf(gnuplot_pipe, "set xlabel 'X'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Y'\n");
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "plot '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", position_vector(0), position_vector(1));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                } else if (num_dims == 3)
                {
                    fprintf(gnuplot_pipe, "set title 'Trajectory'\n");
                    fprintf(gnuplot_pipe, "set xlabel 'X'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Y'\n");
                    fprintf(gnuplot_pipe, "set zlabel 'Z'\n");
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set zrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set view 60, 30\n");
                    fprintf(gnuplot_pipe, "splot '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                    {
                        fprintf(gnuplot_pipe, "%f %f %f\n", position_vector(0), position_vector(1), position_vector(2));
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

            void visualize(const std::vector<spl::VectorRepresentation>& q, const double& u) override
            {

                size_t num_dims = this->trajectory.pos[0].cols();
                fprintf(gnuplot_pipe, "set terminal qt size 800, 1200\n");
                fprintf(gnuplot_pipe, "clear\n");
                fprintf(gnuplot_pipe, "set multiplot layout %zu,1 title 'gnuplot Visualizer' font ',12'\n", (num_dims == 2 || num_dims == 3) ? num_dims + 1 : num_dims);

                for (size_t i = 0; i < num_dims; ++i)
                {
                    fprintf(gnuplot_pipe, "set title 'Dimension %zu vs Time'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xlabel 'Time (s)'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Dimension %zu'\n", i + 1);
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                    fprintf(gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                    for (size_t j = 0; j < q.size(); ++j)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", u, q[j](i));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                    for (size_t j = 0; j < this->trajectory.pos.size(); ++j)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", this->u[j], this->trajectory.pos[j](i));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                }

                if (num_dims == 2)
                {
                    fprintf(gnuplot_pipe, "set title 'Trajectory'\n");
                    fprintf(gnuplot_pipe, "set xlabel 'X'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Y'\n");
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                    fprintf(gnuplot_pipe, "     '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& joint_position : q)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", joint_position(0), joint_position(1));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                    for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                    {
                        fprintf(gnuplot_pipe, "%f %f\n", position_vector(0), position_vector(1));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                } else if (num_dims == 3)
                {
                    fprintf(gnuplot_pipe, "set title 'Trajectory'\n");
                    fprintf(gnuplot_pipe, "set xlabel 'X'\n");
                    fprintf(gnuplot_pipe, "set ylabel 'Y'\n");
                    fprintf(gnuplot_pipe, "set zlabel 'Z'\n");
                    fprintf(gnuplot_pipe, "set xrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set yrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set zrange [-15.0:15.0]\n");
                    fprintf(gnuplot_pipe, "set view 60, 30\n");
                    fprintf(gnuplot_pipe, "splot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                    fprintf(gnuplot_pipe, "      '-' with lines lw 2 lc rgb 'magenta' title 'Path'\n");
                    for (const spl::VectorRepresentation& joint_position : q)
                    {
                        fprintf(gnuplot_pipe, "%f %f %f\n", joint_position(0), joint_position(1), joint_position(2));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                    for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                    {
                        fprintf(gnuplot_pipe, "%f %f %f\n", position_vector(0), position_vector(1), position_vector(2));
                    }
                    fprintf(gnuplot_pipe, "e\n");
                }

                fprintf(gnuplot_pipe, "unset multiplot\n");
                fflush(gnuplot_pipe);

            }

    };
}