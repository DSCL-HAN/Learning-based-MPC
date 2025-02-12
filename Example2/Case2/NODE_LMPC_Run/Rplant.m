function dx=Rplant(x,t)
W1 = 1;
W2 = 1;
W3 = 1;

dx(1) = W1*x(2);
dx(2) = W2*(W3-x(1)^2)*x(2) - x(1);
dx = dx';
end
