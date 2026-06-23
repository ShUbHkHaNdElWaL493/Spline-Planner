/*
    Shubh Khandelwal
*/

#pragma once
#include <stdexcept>
#include "visualizer.hpp"

namespace splvisualizer
{
    class GnuplotVisualizer : public Visualizer
    {

        private:

            FILE* pipe_multiplot;
            FILE* pipe_visualizer;

        public:

            GnuplotVisualizer(const std::pair<std::vector<double>, spl::Trajectory>& path) :
            Visualizer(path),
            pipe_multiplot(popen("gnuplot", "w"))
            {

                if (!pipe_multiplot)
                {
                    std::runtime_error("Failed to open pipe_multiplot.");
                }

                size_t num_dims = this->trajectory.pos[0].cols();

                if (num_dims == 2 || num_dims == 3)
                {
                    pipe_visualizer = popen("gnuplot", "w");
                    if (!pipe_visualizer)
                    {
                        std::runtime_error("Failed to open pipe_visualizer.");
                    }
                }

                fprintf(pipe_multiplot, "set terminal qt size 800, 1200\n");
                fprintf(pipe_multiplot, "set multiplot layout %zu,1 title 'gnuplot Visualizer' font ',12'\n", num_dims);

                for (size_t i = 0; i < num_dims; ++i)
                {
                    fprintf(pipe_multiplot, "set title 'Dimension %zu vs Time'\n", i + 1);
                    fprintf(pipe_multiplot, "set xlabel 'Time (s)'\n");
                    fprintf(pipe_multiplot, "set ylabel 'Dimension %zu'\n", i + 1);
                    fprintf(pipe_multiplot, "set xrange [0.0:20.0]\n");
                    fprintf(pipe_multiplot, "set yrange [-2.0:2.0]\n");
                    fprintf(pipe_multiplot, "plot '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                    for (size_t j = 0; j < this->u.size(); ++j)
                    {
                        fprintf(pipe_multiplot, "%f %f\n", this->u[j], this->trajectory.pos[j](i));
                    }
                    fprintf(pipe_multiplot, "e\n");
                }

                fprintf(pipe_multiplot, "unset multiplot\n");
                fflush(pipe_multiplot);

            }

            ~GnuplotVisualizer()
            {
                if (pipe_multiplot)
                {
                    pclose(pipe_multiplot);
                }
                if (pipe_visualizer)
                {
                    pclose(pipe_visualizer);
                }
            }

            void visualize(const std::vector<spl::VectorRepresentation>& q) override
            {

                size_t num_dims = this->trajectory.pos[0].cols();
                
                // fprintf(pipe_multiplot, "set terminal qt size 800, 1200\n");
                // fprintf(pipe_multiplot, "set multiplot layout %zu,1 title 'gnuplot Visualizer' font ',12'\n", num_dims);

                // for (size_t i = 0; i < num_dims; ++i)
                // {
                //     fprintf(pipe_multiplot, "set title 'Dimension %zu vs Time'\n", i + 1);
                //     fprintf(pipe_multiplot, "set xlabel 'Time (s)'\n");
                //     fprintf(pipe_multiplot, "set ylabel 'Dimension %zu'\n", i + 1);
                //     fprintf(pipe_multiplot, "set yrange [-2.0:2.0]\n");
                //     fprintf(pipe_multiplot, "plot '-' with lines lw 2 lc rgb 'black' title 'Trajectory', \\\n");
                //     fprintf(pipe_multiplot, "     '-' with lines lw 2 lc rgb 'red' title 'D%zu(t)'\n", i + 1);
                //     for (size_t j = 0; j < this->u.size(); ++j)
                //     {
                //         fprintf(pipe_multiplot, "%f %f\n", this->u[j], q.back()(i));
                //     }
                //     fprintf(pipe_multiplot, "e\n");
                //     for (size_t j = 0; j < this->u.size(); ++j)
                //     {
                //         fprintf(pipe_multiplot, "%f %f\n", this->u[j], this->trajectory.pos[j](i));
                //     }
                //     fprintf(pipe_multiplot, "e\n");
                // }

                // fprintf(pipe_multiplot, "unset multiplot\n");
                // fflush(pipe_multiplot);

                if (pipe_visualizer)
                {

                    fprintf(pipe_visualizer, "set terminal qt size 800, 800 title 'gnuplot Visualizer' font ',12'\n");
                    fprintf(pipe_visualizer, "set xlabel 'X'\n");
                    fprintf(pipe_visualizer, "set ylabel 'Y'\n");
                    fprintf(pipe_visualizer, "set xrange [-2.0:2.0]\n");
                    fprintf(pipe_visualizer, "set yrange [-2.0:2.0]\n");

                    if (num_dims == 2)
                    {
                        fprintf(pipe_visualizer, "plot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(pipe_visualizer, "     '-' with lines lw 2 lc rgb 'red' title 'Path'\n");
                        for (const spl::VectorRepresentation& joint_position : q)
                        {
                            fprintf(pipe_visualizer, "%f %f\n", joint_position(0), joint_position(1));
                        }
                        fprintf(pipe_visualizer, "e\n");
                        for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                        {
                            fprintf(pipe_visualizer, "%f %f\n", position_vector(0), position_vector(1));
                        }
                        fprintf(pipe_visualizer, "e\n");
                    } else if (num_dims == 3)
                    {
                        fprintf(pipe_visualizer, "set zlabel 'Z'\n");
                        fprintf(pipe_visualizer, "set zrange [-2.0:2.0]\n");
                        fprintf(pipe_visualizer, "splot '-' with linespoints lw 4 pt 7 ps 1.5 lc rgb 'black' title 'Links', \\\n");
                        fprintf(pipe_visualizer, "      '-' with lines lw 2 lc rgb 'red' title 'Path'\n");
                        for (const spl::VectorRepresentation& joint_position : q)
                        {
                            fprintf(pipe_visualizer, "%f %f %f\n", joint_position(0), joint_position(1), joint_position(2));
                        }
                        fprintf(pipe_visualizer, "e\n");
                        for (const spl::VectorRepresentation& position_vector : this->trajectory.pos)
                        {
                            fprintf(pipe_visualizer, "%f %f %f\n", position_vector(0), position_vector(1), position_vector(2));
                        }
                        fprintf(pipe_visualizer, "e\n");
                    }

                    fflush(pipe_visualizer);

                }

            }

    };
}