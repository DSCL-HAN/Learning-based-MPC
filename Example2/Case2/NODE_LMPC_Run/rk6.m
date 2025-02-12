function dx=rk6(x,u,xr,T)
k1=Nplant(x,u,xr)*T;
k2=Nplant(x+k1*0.5,u,xr)*T;
k3=Nplant(x+k2*0.5,u,xr)*T;
k4=Nplant(x+k3,u,xr)*T;
dx=x + ((k1+k4)/6+(k2+k3)/3);
end