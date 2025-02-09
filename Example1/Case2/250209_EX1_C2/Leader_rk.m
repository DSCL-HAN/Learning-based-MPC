function dx = Leader_rk(x, u, T)
k1 = Leader_plant(x, u)*T;
k2 = Leader_plant(x+k1*0.5, u)*T;
k3 = Leader_plant(x+k2*0.5, u)*T;
k4 = Leader_plant(x+k3, u)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end