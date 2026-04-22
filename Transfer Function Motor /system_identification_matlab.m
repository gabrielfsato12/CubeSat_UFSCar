data = Ensaio1;
t = data.t_ms / 1000;
u = 255 - data.pwm;      % inverter o PWM
y = data.rpm;
Ts = mean(diff(t));


data_motor = iddata(y, u, Ts);
