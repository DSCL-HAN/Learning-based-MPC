function dx=NNplant(x,t)
global wt_1 wt_2 wt_3 wt_4 
dx(1) = wt_1*x(2);
dx(2) = wt_2*x(1) + wt_3*x(2) + wt_4*sin(t*0.5);
dx = dx';
end