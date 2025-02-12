function dxdt = Follower_plant(x,u)
vf = u(1);
wf = u(2);

dxdt(1) = vf*cos(x(3));
dxdt(2) = vf*sin(x(3));
dxdt(3) = wf;

dxdt = dxdt';
end