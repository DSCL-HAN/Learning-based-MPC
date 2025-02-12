function dx=rk7(x,t,T)
k1=Rplant(x,t)*T;
k2=Rplant(x+k1*0.5,t)*T;
k3=Rplant(x+k2*0.5,t)*T;
k4=Rplant(x+k3,t)*T;
dx=x + ((k1+k4)/6+(k2+k3)/3);
end