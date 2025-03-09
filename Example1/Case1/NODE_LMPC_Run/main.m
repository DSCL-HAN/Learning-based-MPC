%% Example 1 - Case 1 - Neural ODE based LMPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025 Seungyong Han, Jeonbuk National University
%
% This code is the intellectual property of Seungyong Han
% at Jeonbuk National University. Users are granted permission 
% to use, copy, modify, and distribute this code for academic or 
% research purposes, provided that this copyright notice is retained 
% in all copies or derivative works.
%
% By using this code, you agree that any resulting publications or 
% presentations will acknowledge its origin.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all
warning('off','all')

%% Define Simulation Parameters 
global wt_1 wt_2 wt_3 wt_4 
load('Ex1_Case1_W1.mat')
wt_1 = data(end);
load('Ex1_Case1_W2.mat')
wt_2 = data(end);
load('Ex1_Case1_W3.mat')
wt_3 = data(end);
load('Ex1_Case1_W4.mat')
wt_4 = data(end);

Q = 50*[1, 0, 0;
     0, 1, 0;
     0, 0, 1];
  
R = 0.1*[1, 0;
    0, 1];

N = 5;
ti = 0.05;
u_h = 0.4;
t = 0;
num_time_u = round((u_h)/ti);
sim_period = ti;
final = 50;
sam_t = 0:sim_period:final;
sample_size = size(sam_t, 2);

Leader_wmr(:,1)=[0.1, 0.1, 0]';
Follower_wmr(:,1)=[0; 0; 0];         
leader_vel(:,1) = [0.6;0.3];
real_leader_vel(:,1) = [0.6;0.3];
s = 1;

%% Run Main Simulation
for i = 1:sample_size
    xe = cos(Follower_wmr(3,i))*(Leader_wmr(1,i)-Follower_wmr(1,i)) + sin(Follower_wmr(3,i))*(Leader_wmr(2,i)-Follower_wmr(2,i));
    ye = -sin(Follower_wmr(3,i))*(Leader_wmr(1,i)-Follower_wmr(1,i)) + cos(Follower_wmr(3,i))*(Leader_wmr(2,i)-Follower_wmr(2,i));
    thetae = (Leader_wmr(3,i)-Follower_wmr(3,i));
    Xe = [xe;ye;thetae];
    
    t0 = (i-1)*ti;
    t1 = (i-1)*ti+ti;
    t_interval = [t0, t1];

    real_leader_vel(:,i+1) = real_Leader_vel_rk(real_leader_vel(:,i),(i-1)*ti,ti);
    leader_vel(:,i+1) = Leader_vel_rk(leader_vel(:,i),t_interval,ti);
    
    if (i == 1)
        [~, uPred] = solve_CFTOCP(Xe,leader_vel(:,i), N, Q, R, ti, u_h);
        u_LMPC(:,1) = uPred(1:2,1);
        sam_u = u_LMPC(:,1)

    elseif (t == num_time_u) 
        tic
        [~, uPred] = solve_CFTOCP(Xe,leader_vel(:,i), N, Q, R, ti, u_h);
        save_time(:,s) = toc;
        s = s + 1;
        u_LMPC(:,i) = uPred(1:2,1);
        sam_u = u_LMPC(:,i)
        t = 0;
    end

    Leader_wmr(:,i+1)=Leader_rk(Leader_wmr(:,i),real_leader_vel(:,i),ti);
    Follower_wmr(:,i+1)=Follower_rk(Follower_wmr(:,i),sam_u,ti);
    conti_u(:,i) = sam_u;
    Xe_data(:,i) = Xe;
    t = t + 1;
end

%% Plot Leader & Follwer WMRs State
figure()
plot3(Leader_wmr(1,:),Leader_wmr(2,:),Leader_wmr(3,:),'b','LineWidth',1.5);
hold on
plot3(Follower_wmr(1,:),Follower_wmr(2,:),Follower_wmr(3,:),'r--','LineWidth',1.5);
grid on
ylabel('$x_{1}(t)$','interpreter','latex', 'FontSize', 15);
xlabel('$x_{2}(t)$','interpreter','latex', 'FontSize', 15);
zlabel('$x_{3}(t)$','interpreter','latex', 'FontSize', 15);
hl = legend('Leader','Follower', 'FontSize', 15);
set(hl, 'Interpreter','latex');

%% Plot Follwer WMR Control Input
figure()
plot(sam_t(1:size(conti_u,2)),conti_u(1,:),'b','LineWidth',1.5);
hold on
plot(sam_t(1:size(conti_u,2)),conti_u(2,:),'r--','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('Input','interpreter','latex', 'FontSize', 15);
hl = legend('$\hat{u}^{\star [1]}_{1}(t)$', '$\hat{u}^{\star [1]}_{2}(t)$', 'FontSize', 15);
set(hl, 'Interpreter','latex');

%% Plot Real & Estimated Velocities
figure()
plot(leader_vel(1,:),leader_vel(2,:),'b','LineWidth',1.5);
hold on
plot(real_leader_vel(1,:),real_leader_vel(2,:),'r--','LineWidth',1.5);
grid on
ylabel('$w_{l}(t)$','interpreter','latex', 'FontSize', 15);
xlabel('$v_{l}(t)$','interpreter','latex', 'FontSize', 15);
hl = legend('NODE Output','Real Velocity');
set(hl, 'Interpreter','latex');

%% VAF & RMSE
vaf_leader_v = vaf(leader_vel(1,:),real_leader_vel(1,:))
vaf_leader_w = vaf(leader_vel(2,:),real_leader_vel(2,:))
RMSE_x = rms((Leader_wmr(1,:)-Follower_wmr(1,:)))
RMSE_y = rms((Leader_wmr(2,:)-Follower_wmr(2,:)))
RMSE_th = rms((Leader_wmr(3,:)-Follower_wmr(3,:)))

%% DATA SAVE
% time = datestr(now, 'yyyy_mm_dd');
% save(['NODE_noADDinput_MPC=',num2str(N),'_Date_',num2str(time),'.mat'])
