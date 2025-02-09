%% 2 norm of x(t) and root mean sqaured error 
% 24.03.31
clear
close all
clc

load('NODE_noADDinput_MPC=5_u_h=0.4_Date_2024_03_30.mat')
x1_h04 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h04 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h04 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x04 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y04 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th04 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

load('NODE_noADDinput_MPC=5_u_h=0.8_Date_2024_03_30.mat')
x1_h08 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h08 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h08 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x08 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y08 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th08 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

load('NODE_noADDinput_MPC=5_u_h=1.2_Date_2024_03_30.mat')
x1_h12 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h12 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h12 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x12 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y12 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th12 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

load('NODE_noADDinput_MPC=5_u_h=1.6_Date_2024_03_30.mat')
x1_h16 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h16 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h16 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x16 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y16 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th16 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

load('NODE_noADDinput_MPC=5_u_h=2_Date_2024_03_30.mat')
x1_h2 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h2 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h2 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x2 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y2 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th2 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

load('NODE_noADDinput_MPC=5_u_h=2.4_Date_2024_03_30.mat')
x1_h24 = (Leader_wmr(1,1:end-1)-Follower_wmr(1,1:end-1));
x2_h24 = (Leader_wmr(2,1:end-1)-Follower_wmr(2,1:end-1));
x3_h24 = (Leader_wmr(3,1:end-1)-Follower_wmr(3,1:end-1));
rmse_x24 = rms((Leader_wmr(1,:)-Follower_wmr(1,:)));
rmse_y24 = rms((Leader_wmr(2,:)-Follower_wmr(2,:)));
rmse_th24 = rms((Leader_wmr(3,:)-Follower_wmr(3,:)));

%% ||x||
figure()
h1 = plot3(sam_t, 0.4*ones(1,size(sam_t,2)),x1_h04.^2,'r','LineWidth',1.5);
hold on
h2 =plot3(sam_t, 0.4*ones(1,size(sam_t,2)),x2_h04.^2,'b--','LineWidth',1.5);
h3 =plot3(sam_t, 0.4*ones(1,size(sam_t,2)),x3_h04.^2,'m:','LineWidth' ,1.5);

plot3(sam_t, 0.8*ones(1,size(sam_t,2)),x1_h08.^2,'r','LineWidth',1.5);
plot3(sam_t, 0.8*ones(1,size(sam_t,2)),x2_h08.^2,'b--','LineWidth',1.5);
plot3(sam_t, 0.8*ones(1,size(sam_t,2)),x3_h08.^2,'m:','LineWidth' ,1.5);

plot3(sam_t, 1.2*ones(1,size(sam_t,2)),x1_h12.^2,'r','LineWidth',1.5);
plot3(sam_t, 1.2*ones(1,size(sam_t,2)),x2_h12.^2,'b--','LineWidth',1.5);
plot3(sam_t, 1.2*ones(1,size(sam_t,2)),x3_h12.^2,'m:','LineWidth' ,1.5);

plot3(sam_t, 1.6*ones(1,size(sam_t,2)),x1_h16.^2,'r','LineWidth',1.5);
plot3(sam_t, 1.6*ones(1,size(sam_t,2)),x2_h16.^2,'b--','LineWidth',1.5);
plot3(sam_t, 1.6*ones(1,size(sam_t,2)),x3_h16.^2,'m:','LineWidth' ,1.5);

plot3(sam_t, 2.0*ones(1,size(sam_t,2)),x1_h2.^2,'r','LineWidth',1.5);
plot3(sam_t, 2.0*ones(1,size(sam_t,2)),x2_h2.^2,'b--','LineWidth',1.5);
plot3(sam_t, 2.0*ones(1,size(sam_t,2)),x3_h2.^2,'m:','LineWidth' ,1.5);

plot3(sam_t, 2.4*ones(1,size(sam_t,2)),x1_h24.^2,'r','LineWidth',1.5);
plot3(sam_t, 2.4*ones(1,size(sam_t,2)),x2_h24.^2,'b--','LineWidth',1.5);
plot3(sam_t, 2.4*ones(1,size(sam_t,2)),x3_h24.^2,'m:','LineWidth' ,1.5);

% p1.MarkerIndices = 1:30:length(sam_t(1,:));
grid on
xlabel('Time (sec)','interpreter','latex', 'FontSize', 15);
% zlabel('','interpreter','latex', 'FontSize', 15);
ylabel('The allowable maximum sampling interval $\bar{\pi}$','interpreter','latex', 'FontSize', 15);
xlim([0 50])
% ylim([0 5])
yticks([0.4 0.8 1.2 1.6 2.0 2.4])
yticklabels({'0.4','0.8','1.2','1.6','2.0','2.4'})
plt_h = legend([h1, h2, h3],{'$||x_{L}-x_{F}||^{2}$','$||y_{L}-y_{F}||^{2}$','$||\theta_{L}-\theta_{F}||^{2}$'}, 'FontSize', 15)
% hold on 
% legend(h2,'$x_{i}^2$ for $h=1$', 'FontSize', 15)
% hl = legend('$x_{i}^2$ for $h=1$','$x_{i}^2$ for $h=2$','$x_{i}^2$ for $h=3$','$x_{i}^2$ for $h=4$','$x_{i}^2$ for $h=4.5$', 'FontSize', 15);
set(plt_h, 'Interpreter','latex');

%% RMSE
figure()
h1 = plot(0.4, rmse_x04,'go','LineWidth',1.5);
hold on
h2 = plot(0.4, rmse_y04,'msquare','LineWidth',1.5);
h3 = plot(0.4, rmse_th04,'r^','LineWidth',1.5);
plot(0.8, rmse_x08,'go','LineWidth',1.5);
plot(0.8, rmse_y08,'msquare','LineWidth',1.5);
plot(0.8, rmse_th08,'r^','LineWidth',1.5);
plot(1.2, rmse_x12,'go','LineWidth',1.5);
plot(1.2, rmse_y12,'msquare','LineWidth',1.5);
plot(1.2, rmse_th12,'r^','LineWidth',1.5);
plot(1.6, rmse_x16,'go','LineWidth',1.5);
plot(1.6, rmse_y16,'msquare','LineWidth',1.5);
plot(1.6, rmse_th16,'r^','LineWidth',1.5);
plot(2.0, rmse_x2,'go','LineWidth',1.5);
plot(2.0, rmse_y2,'msquare','LineWidth',1.5);
plot(2.0, rmse_th2,'r^','LineWidth',1.5);
plot(2.4, rmse_x24,'go','LineWidth',1.5);
plot(2.4, rmse_y24,'msquare','LineWidth',1.5);
plot(2.4, rmse_th24,'r^','LineWidth',1.5);

legend([h1, h2, h3], {'RMSE of $x_{L}$ and $x_{F}$', 'RMSE of $y_{L}$ and $y_{F}$','RMSE of $\theta_{L}$ and $\theta_{F}$'},'interpreter','latex', 'FontSize', 12);
grid on
xlim([0.2 2.6])
ylim([0 0.5])
% list = ['';'Th. 1';'Th. 2';'3';'4'];
yticks([0.1 0.2 0.3 0.4])
yticklabels({'0.1', '0.2','0.3','0.4'})
xticks([0.4 0.8 1.2 1.6 2.0 2.4])
xticklabels({'0.4', '0.8','1.2','1.6','2.0','2.4'})
xlabel('The allowable maximum sampling interval $\bar{\pi}$','interpreter','latex', 'FontSize', 15)
ylabel('Root mean squared error','interpreter','latex', 'FontSize', 15)
% set(gca,'YLim',[0 6]);
% set(gca,'YTick',1:6);
% set(gca, 'YTickLabel', {'','','','','',''});
% set(gca,'TickLabelInterpreter','latex')






