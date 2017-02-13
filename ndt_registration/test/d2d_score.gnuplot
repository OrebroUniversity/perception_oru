reset

set terminal postscript eps size 3.5,2.62 enhanced color font 'Helvetica,12' lw 1
set output 'd2d_score.eps'


set title "Objective score values"
set xlabel "x (m)"
set ylabel "y (m)"
#set pointsize 0.5
set key top right 

set pm3d map
#set cbrange [-2000:0]
splot 'scores.dat' using 1:2:3 with pm3d,\
      'scores.T.gt' using 1:2:(0.0) with points pt 1 title 'GT',\
      'scores.T.d2d' using 1:2:(0.0) with points pt 2 title 'd2d',\
      'scores.T.d2d_sc' using 1:2:(0.0) with points pt 3 title 'd2d_sc',\
      'scores.T.filter' using 1:2:(0.0) with points pt 4 title 'filter',\
      'scores.T.odom' using 1:2:(0.0) with points pt 5 title 'odom',\
#pause -1
	