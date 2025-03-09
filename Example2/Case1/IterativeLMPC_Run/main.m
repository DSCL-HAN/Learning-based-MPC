%% Example 2 - Case 1 - Iterative LMPC
% Reference:
% U. Rosolia and F. Borrelli, 
% "Learning Model Predictive Control for Iterative Tasks. A Data-Driven Control Framework," 
% in IEEE Transactions on Automatic Control, vol. 63, no. 7, pp. 1883-1896, July 2018
%
clc
clear all
close all
warning('off','all')

%% Define Simulation Parameters
global a b c
a = 5.6794;
b = 1.4730;
c = 1.7985;

Q = 50*[1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
  
R = 0.1*[1, 0;
    0, 1];

LMPC_options.solver  = 'sedumi'; % Options are 'gurobi' or 'quadprog'. IMPORTANT: Use gurobi for better precision;
LMPC_options.norm    = 2;        % Options 1-norm or 2-norm;
LMPC_options.goalSet = 0;        % Options 0 => xf = origin, 1 xf = set around origin;
N = 10;
ti = 0.05;
Iterations = 5;
N_CFTOCP = 100;
invariantGoalSet = [];

%% Load the first feasible solution
load('feasibleSolution.mat')

%% ========================================================================
%  ======================= Now Run Learning MPC ===========================
%  ========================================================================
%% Initialize Safe Set and Q-funtion
ref_q = [ref_q';zeros(2,1)];
s_x_feasible = [x_feasible];
s_u_feasible = [u_feasible];
x_cl{1} = s_x_feasible;                          % Safe set vector: Vector collecting the state of the performed iterations
u_cl{1} = s_u_feasible;                         % Safe Input set vector: Vector collecting the input of the performed iterations
Qfun = ComputeCost(s_x_feasible-ref_q, s_u_feasible, Q, R); % Q-function vector: Vector collecting the cost-to-go of the stored states
IterationCost{1} = Qfun;
%% Now run the LMPC
x0 = x_cl{1}(:,1);
[ x_LMPC, u_LMPC, x_cl, u_cl, IterationCost, SS] = LMPC(x0, x_cl, u_cl, IterationCost, Q, R, N, Iterations, LMPC_options,ti,ref_q);

%% DATA SAVE
time = datestr(now, 'yyyy_mm_dd');
save(['Iterative_LMPC_Date_',num2str(time)])
