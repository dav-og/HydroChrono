set terminal pdf 
set output "oswec_decay.pdf"

set samples 100000
set grid
set title "Oswec Pitch Decay Test"
# set lmargin at screen 0.125
set ylabel "Pitch (Radians)"
set xlabel "Time (s)"
set xrange [0:500]
plot "oswec_decay.txt" using 1:2 with lines title "Chrono",\
     "wecsim_comp.txt" using 1:2 with lines title " WECSim"