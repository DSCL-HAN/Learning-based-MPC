function dx = Follower_rk(x, u, T)
k1 = Follower_plant(x, u)*T;
k2 = Follower_plant(x+k1*0.5, u)*T;
k3 = Follower_plant(x+k2*0.5, u)*T;
k4 = Follower_plant(x+k3, u)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end