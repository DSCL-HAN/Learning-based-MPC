function dx=Nplant(x,u,xr)
global a b c
M = [a, b*cos(x(2)-x(1));
    b*cos(x(2)-x(1)), c];
C = [-b*x(4)^2*sin(x(2)-x(1));
    b*x(3)^2*sin(x(2)-x(1))];

dx(1) = x(3);
dx(2) = x(4);
dx(3:4) = inv(M)*(-C+u+xr);
dx = dx';


