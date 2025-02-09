clear all
close all
clc
%% Loss
load('240330WMR_RNN_loss.mat')
loss_RNN = loss;
load('240330WMR_NODEloss.mat')
loss_NODE = data;

figure()
h1 = plot(loss_NODE(1,:),'-r','LineWidth',1.5);
hold on
h2 = plot(loss_RNN(1,:),'b:','LineWidth',1.5);
grid on
xlabel('Epoch','interpreter','latex', 'FontSize', 15);
ylabel('Loss function','interpreter','latex', 'FontSize', 15);
xlim([0 1000])
% hl = legend('RNN','NODEs', 'FontSize', 15);
% set(hl, 'Interpreter','latex');
plt_h = legend([h2, h1],{'RNN','NODEs'}, 'FontSize', 15)
set(plt_h, 'Interpreter','latex');
%% RNN, NODE
load('240330WMR_RNN_w1.mat')
load('240330WMR_RNN_w2.mat')
load('240330WMR_RNN_xpred.mat')
load('240330WMR_RNN_xreal.mat')
load('NODE_noADDinput_MPC=5_u_h=0.4_Date_2024_03_30.mat')
% figure()
% plot(xreal(:,1),xreal(:,2),'k','LineWidth',1.5);
% hold on
% % plot(xpred(1:20:end,1),xpred(1:20:end,2),'--b*','LineWidth',1.5);
% plot(xpred(1:1:end,1),xpred(1:1:end,2),'b*','LineWidth',1.5);
% plot(leader_vel(1,1:50:2000),leader_vel(2,1:50:2000),'ro','LineWidth',1.5);
% % plot(real_leader_vel(1,:),real_leader_vel(2,:),'r','LineWidth',1.5);
% 
% grid on
% xlabel('$\hat{z}_{1}(t)$','interpreter','latex', 'FontSize', 15);
% ylabel('$\hat{z}_{2}(t)$','interpreter','latex', 'FontSize', 15);
% % xlim([-0.6 0.6])
% % ylim([-0.4 0.6])
% hl = legend('True','RNN','NODEs', 'FontSize', 15);
% set(hl, 'Interpreter','latex');

%% 2D
figure()
subplot(2,1,1)
plot(sam_t(1:size(xreal,1)), xreal(:,1),'k','LineWidth',1.5);
hold on
plot(sam_t(1:size(xreal,1)), xpred(1:1:end,1),'b*','LineWidth',1.5);
plot(sam_t(1:size(xreal,1)), leader_vel(1,1:size(xreal,1)),'ro','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$\hat{z}_{1}(t)$','interpreter','latex', 'FontSize', 15);
% xlim([-0.6 0.6])
% ylim([-0.4 0.6])
hl = legend('True','RNN','NODEs', 'FontSize', 15);
set(hl, 'Interpreter','latex');
subplot(2,1,2)
plot(sam_t(1:size(xreal,1)), xreal(:,2),'k','LineWidth',1.5);
hold on
plot(sam_t(1:size(xreal,1)), xpred(1:1:end,2),'b:','LineWidth',1.5);
plot(sam_t(1:size(xreal,1)), leader_vel(2,1:size(xreal,1)),'r--','LineWidth',1.5);
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
ylabel('$\hat{z}_{2}(t)$','interpreter','latex', 'FontSize', 15);
% xlim([-0.6 0.6])
% ylim([-0.4 0.6])
hl = legend('True','RNN','NODEs', 'FontSize', 15);
set(hl, 'Interpreter','latex');

% 
% 
% 
% 
% 
% 
% 
% 
% 
