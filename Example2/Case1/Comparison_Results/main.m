%% Example 2 - Case 1 - Comparison between Proposed LMPC & Iterative LMPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C)
% Authors: Seungyong Han <hansy@jbnu.ac.kr>
% 
% Date: March, 1, 2025
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

%% Data Load
load('w=000175_NODE_MPC_Date_2024_04_07.mat')
x_lmpc_node = x;
LMPC_NODE_Cost = LMPC_NODE_Cost;
LMPC_NODE_u = conti_u;
load('Iterative_LMPC_Date_2025_02_11.mat')
x_lmpc = x_cl{end};
LMPC_Cost = IterationCost;
u_lmpc = u_cl{end};

%% Plot
% State
subplot(2,2,1)
h2 = plot(sam_t,x_lmpc_node(1,1:end),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_lmpc(1,:),'b:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{1}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC ','Proposed LMPC', 'FontSize', 10);
axis([0 30 -0.1 0.1])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,2)
h2 = plot(sam_t,x_lmpc_node(2,1:end),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_lmpc(2,:),'b:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{2}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC','Proposed LMPC', 'FontSize', 10);
axis([0 30 -0.1 0.1])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,3)
h2 = plot(sam_t,x_lmpc_node(3,1:end),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_lmpc(3,:),'b:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{3}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC','Proposed LMPC', 'FontSize', 10);
axis([0 30 -0.1 0.1])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,4)
h2 = plot(sam_t,x_lmpc_node(4,1:end),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_lmpc(4,:),'b:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{4}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC','Proposed LMPC', 'FontSize', 10);
axis([0 30 -0.1 0.1])
set(aa, 'Interpreter','latex');
grid on
box on

% Control Input 1
figure()
h2 = plot(sam_t(1:size(conti_u,2)),LMPC_NODE_u(1,:),'r','LineWidth',1.5);
hold on 
h1 = plot(sam_t(1:size(conti_u,2)),u_lmpc(1,:),'b:','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$u_{1}(t)$','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC ','Proposed LMPC', 'FontSize', 15);
set(aa, 'Interpreter','latex');
axis([0 30 -2 1])
box on

% Control Input 2
figure()
h2 = plot(sam_t(1:size(conti_u,2)),LMPC_NODE_u(2,:),'r','LineWidth',1.5);
hold on 
h1 = plot(sam_t(1:size(conti_u,2)),u_lmpc(2,:),'b:','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$u_{2}(t)$','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC ','Proposed LMPC', 'FontSize', 15);
set(aa, 'Interpreter','latex');

axis([0 30 -0.5 1.5])
box on

% Cost
figure()
iter = [1 2 3 4 5]
lmpc_cost = [26.2666 26.2666 26.2666 26.2666 26.2666]
ILMPC_Cost = [LMPC_Cost{2}(1) LMPC_Cost{3}(1) LMPC_Cost{4}(1) LMPC_Cost{5}(1) LMPC_Cost{6}(1)]
h1 = plot(iter,ILMPC_Cost,'bo--','LineWidth',1.5);
hold on
h2 = plot(iter,lmpc_cost,'ro-','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('Iterative control performance','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'Iterative LMPC','Proposed LMPC', 'FontSize', 10);
xticks([1 2 3 4 5])
xticklabels({'1','2','3','4','5'});
set(aa, 'Interpreter','latex');
grid on
box on
