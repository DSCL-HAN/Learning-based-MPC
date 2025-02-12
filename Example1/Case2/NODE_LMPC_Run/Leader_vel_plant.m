function dxdt = Leader_vel_plant(x,t)
global W1 W2 W3 B1 B2 B3

node_f = [x(1) x(2) t x(1)^2 x(1)*x(2)]';

Layer1 = tanh(W1*node_f+B1');
Layer2 = tanh(W2*Layer1+B2');
Layer3 = W3*Layer2+B3';

dxdt = Layer3;
end