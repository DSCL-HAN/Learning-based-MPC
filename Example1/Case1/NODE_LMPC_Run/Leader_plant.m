function dxdt = Leader_plant(x,u)
vl = u(1);
wl = u(2);

dxdt(1) = vl*cos(x(3));
dxdt(2) = vl*sin(x(3));
dxdt(3) = wl;

dxdt = dxdt';
end