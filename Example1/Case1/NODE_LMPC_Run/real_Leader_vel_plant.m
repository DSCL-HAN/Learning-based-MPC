function dxdt = real_Leader_vel_plant(x,t)
dxdt(1) = -0.2*x(1) -1.5*x(2);
dxdt(2) = 0.5*x(1) -0.1*cos(t*0.5);

dxdt = dxdt';
end