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

%% Define Simulation Parameters
global a b c wt_1 wt_2 wt_3 wt_4 
load('Ex2_Case1_W1_250211.mat')
wt_1 = double(data(end));
load('Ex2_Case1_W2_250211.mat')
wt_2 = double(data(end));
load('Ex2_Case1_W3_250211.mat')
wt_3 = double(data(end));
load('Ex2_Case1_W4_250211.mat')
wt_4 = double(data(end));

Q = 50*[1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
  
R = 0.1*[1, 0;
    0, 1];

a = 5.6794;
b = 1.4730;
c = 1.7985;
N = 10;
ti = 0.05;
u_h = 0.1;
t = 0;
num_time_u = round((u_h)/ti);
sim_period = ti;
final = 30;
sam_t = 0:sim_period:final;
sample_size = size(sam_t, 2);

x(:,1)=[0.1, -0.1, 0, 0]';
xhat(:,1)=[0.1, -0.1, 0, 0]';
z(:,1) = [0.1, 0]';
z_real(:,1) = [0.1, 0]';
s = 1;

%% Run Main Simulation
for i = 1:sample_size
        
    if (i == 1)
        [~, uPred] = solve_CFTOCP(x(:,1), z(:,1), N, Q, R, ti, u_h);
        u_LMPC(:,1) = uPred(1:2,1);
        sam_u = u_LMPC(:,1);

        [~, uPredhat] = solve_CFTOCP(xhat(:,1), z(:,1), N, Q, R, ti, u_h);
        u_LMPChat(:,1) = uPredhat(1:2,1);
        sam_uhat = u_LMPChat(:,1);

    elseif (t == num_time_u) 
        tic
        [~, uPred] = solve_CFTOCP(x(:,i), z(:,i), N, Q, R, ti, u_h);
        save_time(:,s) = toc;
        s = s + 1;
        u_LMPC(:,i) = uPred(1:2,1);
        sam_u = u_LMPC(:,i)
        
        [~, uPredhat] = solve_CFTOCP(xhat(:,i), z(:,i), N, Q, R, ti, u_h);
        u_LMPChat(:,i) = uPredhat(1:2,1);
        sam_uhat = u_LMPChat(:,i)
        t = 0;
    end
    z_real(1,i) = z_real(1,i);
    z_real(2,i) = z_real(2,i);
  
    z(:,i+1) = rk5(z(:,i),i*ti,ti);
    z_real(:,i+1) = rk7(z_real(:,i),i*ti,ti);
    
    x(:,i+1) = rk6(x(:,i),sam_u,z_real(:,i),ti);
    xhat(:,i+1) = rk6(xhat(:,i),sam_uhat,z(:,i),ti);
    
    M = [a, b*cos(x(2,i)-x(1,i));
        b*cos(x(2,i)-x(1,i)), c];
    C = [-b*x(4,i)^2*sin(x(2,i)-x(1,i));
        b*x(3,i)^2*sin(x(2,i)-x(1,i))];

    Mhat = [a, b*cos(xhat(2,i)-xhat(1,i));
        b*cos(xhat(2,i)-xhat(1,i)), c];
    Chat = [-b*xhat(4,i)^2*sin(xhat(2,i)-xhat(1,i));
        b*xhat(3,i)^2*sin(xhat(2,i)-xhat(1,i))];

    temp_f(:,i) =  [x(3,i);x(4,i);inv(M)*(-C+sam_u+z_real(:,i))]; 
    temp_fhat(:,i) = [xhat(3,i);xhat(4,i);inv(Mhat)*(-Chat+sam_uhat+z(:,i))]; 
    temp_omega(:,i) = sqrt((temp_f(:,i)-temp_fhat(:,i))'*(temp_f(:,i)-temp_fhat(:,i)));
    temp_upper(:,i) = sqrt((x(:,i)-xhat(:,i))'*(x(:,i)-xhat(:,i)));
    conti_u(:,i) = sam_u;
    
    t = t + 1;
    if i*ti == 1
        disp('1sec')
    elseif i*ti == 20
        disp('20sec')
    end
end
i = i + 1;
M = [a, b*cos(x(2,i)-x(1,i));
    b*cos(x(2,i)-x(1,i)), c];
C = [-b*x(4,i)^2*sin(x(2,i)-x(1,i));
    b*x(3,i)^2*sin(x(2,i)-x(1,i))];

Mhat = [a, b*cos(xhat(2,i)-xhat(1,i));
    b*cos(xhat(2,i)-xhat(1,i)), c];
Chat = [-b*xhat(4,i)^2*sin(xhat(2,i)-xhat(1,i));
    b*xhat(3,i)^2*sin(xhat(2,i)-xhat(1,i))];

temp_f(:,i) =  [x(3,i);x(4,i);inv(M)*(-C+sam_u+z_real(:,i))]; % fhat
temp_fhat(:,i) = [xhat(3,i);xhat(4,i);inv(Mhat)*(-Chat+sam_uhat+z(:,i))]; 
temp_omega(:,i) = sqrt((temp_f(:,i)-temp_fhat(:,i))'*(temp_f(:,i)-temp_fhat(:,i)));
temp_upper(:,i) = sqrt((x(:,i)-xhat(:,i))'*(x(:,i)-xhat(:,i)));

%% DATA SAVE
% time = datestr(now, 'yyyy_mm_dd');
% save(['w=000175_NODE_MPC_Date_',num2str(time)])
