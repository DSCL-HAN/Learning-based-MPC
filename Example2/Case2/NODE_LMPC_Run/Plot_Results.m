%% Example 2 - Case 1 - Neural ODE based LMPC
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

%% Data Load
load('w=00350_NODE_MPC_Date_2025_02_11.mat')

%% Trajectories of the error of the time-varying parameter’s dynamics and it’s upper bound
figure()
plot(sam_t,temp_omega(:,1:end-1),'b','LineWidth',1.5);
hold on
plot(sam_t,temp_upper(:,1:end-1)*1+0.02,'r--','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('Nonlinear function','interpreter','latex', 'FontSize', 15);
axis([0 final 0 0.04])
grid on
h4 = legend('$||f(x(t),u(t),z(t))-f(\hat{x}(t),\hat{u}(t),\hat{z}(t)||$', '$\lambda_{f}||x(t)-\hat{x}(t)||+f_{max}$', 'FontSize', 15);
set(h4, 'Interpreter','latex');
box on
0.03-0.02-1*(1*0.02+0.005)*(0.1*exp(0.05))

%% Trajectories of xT P x under the ultimate boundedness
for i = 1:sample_size-1
    terminal_set_gam(i) = x(:,i)'*x(:,i);
end
time_s = linspace(0,final, 10);
gam = 0.001;
gam0 = 0.02;
gamd = 0.03;
temp_max = gam*ones(1,10);
temp_0max = gam0*ones(1,10);
temp_dmax = gamd*ones(1,10);

terminal_set_gam(i+1) = x(:,i+1)'*x(:,i+1);
figure()
plot(sam_t, terminal_set_gam,'b','LineWidth',1.5);
hold on
plot(time_s, temp_dmax,'mo--','LineWidth',1.5);
plot(time_s, temp_0max,'ro--','LineWidth',1.5);
plot(time_s, temp_max,'ko--','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
hl = legend('$x^{T}Px$', '$\gamma_{d}$', '$\gamma_{0}$', '$\gamma$', 'FontSize', 15);
set(hl, 'Interpreter','latex');
grid on
box on
% axis([0 30 0 0.035])
gamd-gam0-1*(1*0.02+0.005)*(0.1*exp(0.1))

% figure()
% plot(sam_t, terminal_set_gam,'b','LineWidth',1.5);
% grid on
% box on
% % axis([0 30 0 10^(-5)])
