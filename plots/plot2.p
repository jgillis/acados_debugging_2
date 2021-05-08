# Gnuplot script file for plotting data in file "forplot.txt"
# This file is called gnuplot_script.p
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "Occupancy grid and global path"
set xlabel "X"
set ylabel "Y"
set key left top
plot "occupancy_grid.txt" with points, "local_path.txt" with lines, "bubbles_midpoints.txt" with points
