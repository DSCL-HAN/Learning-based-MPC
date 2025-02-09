function dx = real_Leader_vel_rk(x,t, T)
k1 = real_Leader_vel_plant(x, t) * T;
k2 = real_Leader_vel_plant(x + 0.5*k1, t + 0.5*T) * T;
k3 = real_Leader_vel_plant(x + 0.5*k2, t + 0.5*T) * T;
k4 = real_Leader_vel_plant(x + k3,     t + T)      * T;
dx = x + ((k1 + k4)/6 + (k2 + k3)/3);
end