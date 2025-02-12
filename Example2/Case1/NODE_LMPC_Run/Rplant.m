function dx=Rplant(x,t)

dx(1) = 1*x(2);
dx(2) = -0.8*x(1) -1.5*x(2) + 0.3*sin(t*0.5);
dx = dx';

