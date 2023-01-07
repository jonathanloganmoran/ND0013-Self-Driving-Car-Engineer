set title "PID controller test"
set xlabel "time"
set ylabel "trajectory"
set grid
plot "pid_data.txt" u (column(0)):2 w l title "trajectory","pid_data.txt" u (column(0)):3 w l title "pid correction"