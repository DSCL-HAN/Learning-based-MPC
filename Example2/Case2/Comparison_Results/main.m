%% Example 2 - Case 2 - Comparison between Proposed LMPC & TDMPC
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
load('w=00350_NODE_MPC_Date_2025_02_11.mat')
x_lmpc_node = x;
% LMPC_NODE_Cost = LMPC_NODE_Cost;
LMPC_NODE_u = conti_u;
load('Ex2_Case2_TDMPC_Performance_Test.mat')
x_tdmpc = obs_array';
u_tdmpc = action_array';
load('Ex2_Case2_NODE_Loss.mat')
NODE_loss = Loss;
load('TDMPC_Loss.mat')
TDMPC_loss = TDMPC_Loss(21:end);

%% Plot
% Loss
figure()
subplot(1,2,1)
plot(NODE_loss,'r','LineWidth',1.5);
xlabel('Steps','interpreter','latex', 'FontSize', 15);
ylabel('Loss ','interpreter','latex', 'FontSize', 15);
legend('NODE', 'FontSize', 10);
xlim([0 size(NODE_loss,2)])
% set(aa, 'Interpreter','latex');
grid on
box on

subplot(1,2,2)
h1 = plot(TDMPC_loss,'g:','LineWidth',1.5);
xlabel('Steps','interpreter','latex', 'FontSize', 15);
ylabel('Loss ','interpreter','latex', 'FontSize', 15);
legend('TDMPC', 'FontSize', 10);
xlim([0 size(TDMPC_loss,2)])
% set(aa, 'Interpreter','latex');
grid on
box on

% State
figure()
subplot(2,2,1)
h2 = plot(sam_t,x_lmpc_node(1,1:end-1),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_tdmpc(1,1:end-1),'g:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{1}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC ','Proposed LMPC', 'FontSize', 10);
axis([0 final -2 2])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,2)
h2 = plot(sam_t,x_lmpc_node(2,1:end-1),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_tdmpc(2,1:end-1),'g:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{2}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC','Proposed LMPC', 'FontSize', 10);
axis([0 final -1 1])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,3)
h2 = plot(sam_t,x_lmpc_node(3,1:end-1),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_tdmpc(3,1:end-1),'g:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{3}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC','Proposed LMPC', 'FontSize', 10);
axis([0 final -0.5 0.5])
set(aa, 'Interpreter','latex');
grid on
box on

subplot(2,2,4)
h2 = plot(sam_t,x_lmpc_node(4,1:end-1),'r','LineWidth',1.5);
hold on
h1 = plot(sam_t,x_tdmpc(4,1:end-1),'g:','LineWidth',1.5);
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$x_{4}(t)$ ','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC','Proposed LMPC', 'FontSize', 10);
axis([0 final -0.5 0.5])
set(aa, 'Interpreter','latex');
grid on
box on

% Control Input 1
figure()
h1 = plot(sam_t(1:size(conti_u,2)),u_tdmpc(1,:),'g:','LineWidth',1.0);
hold on 
h2 = plot(sam_t(1:size(conti_u,2)),LMPC_NODE_u(1,:),'r','LineWidth',2);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$u_{1}(t)$','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC','Proposed LMPC', 'FontSize', 15);
set(aa, 'Interpreter','latex');
axis([0 final -5 5])
box on

% Control Input 2
figure()
h1 = plot(sam_t(1:size(conti_u,2)),u_tdmpc(2,:),'g:','LineWidth',1.0);
hold on 
h2 = plot(sam_t(1:size(conti_u,2)),LMPC_NODE_u(2,:),'r','LineWidth',2);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$u_{2}(t)$','interpreter','latex', 'FontSize', 15);
aa = legend([h1, h2],'TDMPC','Proposed LMPC', 'FontSize', 15);
set(aa, 'Interpreter','latex');

axis([0 final -5 5])
box on
