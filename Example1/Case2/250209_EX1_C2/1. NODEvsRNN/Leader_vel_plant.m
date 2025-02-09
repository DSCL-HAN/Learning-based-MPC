function [next_x, next_h]= Leader_vel_plant(x, h)
global w1 w2

xh = [x' h'];
xhw1 = xh*w1;

next_h = [tanh(xhw1(1)), tanh(xhw1(2)), tanh(xhw1(3)), tanh(xhw1(4))];
next_x = next_h*w2;

end