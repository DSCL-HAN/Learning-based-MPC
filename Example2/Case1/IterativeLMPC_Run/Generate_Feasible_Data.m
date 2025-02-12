%% Example 2 - Case 1 - Generate Feasible Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C)
% Authors: Seungyong Han <hansy@jbnu.ac.kr>
% 
% Date: March, 1, 2025
%
% This software/code is the intellectual property of Seungyong Han
% at Jeonbuk National University. It is provided "as is," without any 
% warranty of any kind. Users are granted permission to use, copy, modify, 
% and distribute this code for academic or research purposes, provided 
% that this copyright notice is retained in all copies or derivative works.
%
% By using this code, you agree that any resulting publications or 
% presentations will acknowledge its origin.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all
warning('off','all')

%% Define Simulation Parameters 
global a b c
a = 5.6794;
b = 1.4730;
c = 1.7985;
ref_q = [0 0];
sample_size_ref = length(ref_q);
umax = 1;
sim_period = 0.05;
sim_time =30;
t = 0:sim_period:sim_time;
sample_size = size(t, 2);
Kp=[5 10]'*1;
Kd = sqrt(Kp)*0.5;
Ki = Kp;    
x = [0.1; -0.1; 0; 0;];

%% Run Main Simulation
for i=1:sample_size-1
    q = x(1:2,i);
    qd = x(3:4,i);
    
    e = ref_q' - q;
    ed = 0 - qd;
    temp_ref(:,i) = ref_q;
    M = get_MassMatrix(x(1:2,i));
    C = get_CoriolisVector(x(1:2,i),x(3:4,i));
    
    u = C + M*(Kp.*(e) + Kd.*(ed));
    temp_u(:,i) = u;
    ext(:,i) = [0; 0];

    if(i ~= sample_size)
        x(:,i+1) = simple_rk2(x(:,i),u + ext(:,i), sim_period, M, C);
    end
end
temp_ref(:,i+1) = ref_q;

%% Plot 
figure()
set(gcf,'color','w');
a=1;
for i=1:2
    subplot(2,1,i)
    hold off
    plot(t, temp_ref(i,:),'-k','LineWidth',1.5')
    hold on;
    plot(t, x(i,:),'--r','LineWidth',1.5')
    grid on;
    if i==a
        legend('ref', 'joint degree')
        a=a+1;
    end
   
end

figure()
plot(t(1:end-1),ext(1,:),'r');
hold on
plot(t(1:end-1),ext(2,:),'b');
xlabel('Time')
x_feasible = x(:,1:end-1);
u_feasible = temp_u;

figure()
plot(t(1:end-1), temp_u(1,:),'-k','LineWidth',1.5')
hold on
plot(t(1:end-1), temp_u(2,:),'-k','LineWidth',1.5')
xlabel('Time')
x_feasible = x(:,1:end-1);
u_feasible = temp_u;

%% DATA SAVE
save('feasibleSolution.mat','x_feasible','u_feasible','ref_q')

%% Useful Functions
function dx = simple_rk2(x, u, T, M, C)
k1 = simple_plant2(x, u, M, C)*T;
k2 = simple_plant2(x+k1*0.5, u, M, C)*T;
k3 = simple_plant2(x+k2*0.5, u, M, C)*T;
k4 = simple_plant2(x+k3, u, M, C)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end

function dxdt = simple_plant2(x, u, M, C)
temp_dtu = inv(M)*(-C + u);
% temp_dtu = u;
dxdt = [x(3);x(4);temp_dtu(1);temp_dtu(2)];
end

function out = sat(u)
global umax
if abs(u) <= umax
out = u;
else
out = umax*sign(u);
end
end