function dxdt = real_Leader_vel_plant(x,t)
W_1 = 1;
W_2 = 1;
W_3 = 1;

dxdt(1) = W_1*x(2);
dxdt(2) = W_2*(W_3-x(1)^2)*x(2)-x(1);

dxdt = dxdt';
end