function dxdt = plant(x,u, vel)
vl = vel(1);
wl = vel(2);
dxdt(1) = wl*x(2) - u(1);
dxdt(2) = -wl*x(1) + vl*x(3);
dxdt(3) =  - u(2);
dxdt = dxdt';
end