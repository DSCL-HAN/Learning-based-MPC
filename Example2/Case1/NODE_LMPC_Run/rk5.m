function dx=rk5(x,t,T)
% k1=NNplant(x,t)*T;
% k2=NNplant(x+k1*0.5,t)*T;
% k3=NNplant(x+k2*0.5,t)*T;
% k4=NNplant(x+k3,t)*T;
% dx = x + ((k1+k4)/6+(k2+k3)/3);

k1 = NNplant(x, t)*T;
k2 = NNplant(x + 0.5*k1, t + 0.5*T)*T;
k3 = NNplant(x + 0.5*k2, t + 0.5*T)*T;
k4 = NNplant(x + k3, t + T)*T;
dx = x + ((k1 + k4)/6 + (k2 + k3)/3);
end
