function dxdt = Leader_vel_plant(x,t)
global wt_1 wt_2 wt_3 wt_4 

dxdt(1) = wt_1*x(1) + wt_2*x(2);
dxdt(2) = wt_3*x(1) + wt_4*cos(t*0.5);
dxdt = dxdt';
