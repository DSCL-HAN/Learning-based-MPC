%% Example 1 - Case 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025 Seungyong Han, Jeonbuk National University
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

%% Options
load('NODE_Weight.mat')
global W1 W2 W3 B1 B2 B3

LMPC_options.solver  = 'sedumi'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;
LMPC_options.norm    = 2;        % Options 1-norm or 2-norm;
LMPC_options.goalSet = 0;        % Options 0 => xf = origin, 1 xf = set around origin;

%% Define Simulation Parameters 
Q = 50*[1, 0, 0;
     0, 1, 0;
     0, 0, 1];
  
R = 0.1*[1, 0;
    0, 1];

N = 5;
ti = 0.05;
u_h = 0.2;
t = 0;
num_time_u = round((u_h)/ti);
sim_period = ti;
final = 20;
sam_t = 0:sim_period:final;
sample_size = size(sam_t, 2);

Leader_wmr(:,1)=[0.1, 0.1, 0]';
Follower_wmr(:,1)=[0; 0; 0];         
leader_vel(:,1) = [0.1;0.0];
real_leader_vel(:,1) = leader_vel(:,1);
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

%% Plot Leader & Follwer WMRs State in 3D
figure()
plot3(Leader_wmr(1,:),Leader_wmr(2,:),Leader_wmr(3,:),'b','LineWidth',1.5);
hold on
plot3(Follower_wmr(1,:),Follower_wmr(2,:),Follower_wmr(3,:),'r--','LineWidth',1.5);
grid on
ylabel('$x_{1}(t)$','interpreter','latex', 'FontSize', 15);
xlabel('$x_{2}(t)$','interpreter','latex', 'FontSize', 15);
zlabel('$x_{3}(t)$','interpreter','latex', 'FontSize', 15);
% xlim([-0.6 0.5])
% ylim([0 0.8])
% zlim([0 0.9])
hl = legend('Leader','Follower', 'FontSize', 15);
set(hl, 'Interpreter','latex');

%% Plot Leader & Follwer WMRs State in 2D
figure()
plot(sam_t,leader_vel(1,1:end-1),'b','LineWidth',1.5);
hold on
plot(sam_t,real_leader_vel(1,1:end-1),'r--','LineWidth',1.5);
plot(sam_t,leader_vel(2,1:end-1),'b','LineWidth',1.5);
plot(sam_t,real_leader_vel(2,1:end-1),'r--','LineWidth',1.5);
grid on
ylabel('$w_{l}(t)$','interpreter','latex');
xlabel('$v_{l}(t)$','interpreter','latex');
hl = legend('Leader Velocity','Follower Velocity');
set(hl, 'Interpreter','latex');

vaf_leader_v = vaf(leader_vel(1,:),real_leader_vel(1,:))
vaf_leader_w = vaf(leader_vel(2,:),real_leader_vel(2,:))

figure()
plot(leader_vel(1,:),leader_vel(2,:),'b','LineWidth',1.5);
hold on
plot(real_leader_vel(1,:),real_leader_vel(2,:),'r--','LineWidth',1.5);
grid on
ylabel('$w_{l}(t)$','interpreter','latex');
xlabel('$v_{l}(t)$','interpreter','latex');
hl = legend('Leader Velocity','Follower Velocity');
set(hl, 'Interpreter','latex');

figure()
plot(sam_t(1:size(conti_u,2)),conti_u(1,:),'b','LineWidth',1.5);
hold on
plot(sam_t(1:size(conti_u,2)),conti_u(2,:),'r--','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('Input','interpreter','latex', 'FontSize', 15);
hl = legend('$\hat{u}^{\star [1]}_{1}(t)$', '$\hat{u}^{\star [1]}_{2}(t)$', 'FontSize', 15);
set(hl, 'Interpreter','latex');

%legend('$x_{1}(t)$','$x_{2}(t)$','interpreter','latex');
% 
% 
% figure()
% plot(t,u_opt(1,:),'r','LineWidth',1.5);
% grid on
% ylabel('$u(t)$','interpreter','latex');
% xlabel('Time(sec)');
% hl = legend('$u_{1}(t)$','interpreter','latex');
% set(hl, 'Interpreter','latex');
%legend('$u_{1}(t)$','interpreter','latex');

% figure()
% plot(x_opt(1,:),x_opt(2,:),'r','LineWidth',1.5)
% grid on
% ylabel('$x_{2}(t)$','interpreter','latex')
% xlabel('$x_{1}(t)$','interpreter','latex')
% hl = legend('$x(t)$','interpreter','latex');
% set(hl, 'Interpreter','latex');
% % legend('$x(t)$','interpreter','latex');
% 
% ul_s = size(u_LMPC,2);
% tl = 0:ti:ul_s*ti;
% figure()
% plot(tl,x_LMPC(1,:),'r','LineWidth',1.5);
% hold on
% plot(tl,x_LMPC(2,:),'b','LineWidth',1.5);
% grid on
% ylabel('$x(t)$','interpreter','latex');
% xlabel('Time(sec)');
% hl = legend('$x_{1}(t)$','$x_{2}(t)$');
% set(hl, 'Interpreter','latex');
% 
% figure()
% plot(sam_t(1:size(conti_u,2)),u_LMPC(1,:),'r','LineWidth',1.5);
% grid on
% hold on
% plot(sam_t(1:size(conti_u,2)),u_LMPC(2,:),'b','LineWidth',1.5);
% ylabel('$u(t)$','interpreter','latex');
% xlabel('Time(sec)');
% % hl = legend('$u_{1}(t)$','interpreter','latex');
% set(hl, 'Interpreter','latex');

% %% Display Cost and Plot the Results
% clc
% SS = [];
% for i = 1:(Iterations+1)
%     SS = [SS, x_cl{i}];
%     fprintf('Iteration cost at iteration %d:  %13.4f\n', [i, IterationCost{i}(1)]);
% end
% fprintf('Optimal Cost:  %13.4f\n', [optCost(1)]);

% %%
% figure()
% hold on
% a  = plot(SS(1,:), SS(2,:), 'or');
% aa  = plot(x_feasible(1,:), x_feasible(2,:), '-dm');
% b  = plot(x_LMPC(1,:), x_LMPC(2,:), '-sb');
% c  = plot(x_opt(1,:), x_opt(2,:), '--k*');
% if LMPC_options.goalSet == 1
%     plot(invariantGoalSet,'wire',true)
% end
% xlabel('$$x_1$$', 'interpreter', 'latex','fontsize',20);
% ylabel('$$x_2$$', 'interpreter', 'latex','fontsize',20);
% h = legend([a, aa, b, c], 'Stored data', 'First feaisble trajectory', 'LMPC closed-loop at convergence', 'Optimal solution');
% set(h,'fontsize',16, 'interpreter', 'latex')

% %%
% itCost = [];
% for i = 1:(Iterations+1)
%     itCost = [itCost, IterationCost{i}(1)];
% end
% figure()
% semilogy([1:size(itCost,2)],itCost, '-o')
% hold on
% semilogy([1,size(itCost,2)],[optCost(1), optCost(1)], '-o')
% 
% figure()
% plot([1:size(itCost,2)],itCost, '-o')
% hold on
% plot([1,size(itCost,2)],[optCost(1), optCost(1)], '-o')

RMSE_x = rms((Leader_wmr(1,:)-Follower_wmr(1,:)))
RMSE_y = rms((Leader_wmr(2,:)-Follower_wmr(2,:)))
RMSE_th = rms((Leader_wmr(3,:)-Follower_wmr(3,:)))
%% DATA SAVE
time = datestr(now, 'yyyy_mm_dd');
save(['NODE_noADDinput_MPC=',num2str(N),'_Date_',num2str(time),'.mat'])

%% 
% plotComparisonClosedLoop(2)
