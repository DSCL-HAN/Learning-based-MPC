function [ x_LMPC, u_LMPC, x_cl_out, u_cl_out, IterationCost_out, SS] = LMPC(x0, x_cl, u_cl, IterationCost, Q, R, N, Iterations,LMPC_options, ti,ref_q)
%% Iterative LMPC
% Now start iteration loop
j = 1;
SS = x_cl{1};
uSS = u_cl{1};
Qfun= IterationCost{1};
IterationCost_out{1} = IterationCost{1};
x_cl_out{1} = x_cl{1};
u_cl_out{1} = u_cl{1};
list_x0 = [];

%% Dynamics
import casadi.*
x1 = MX.sym('x1'); % States
x2 = MX.sym('x2');
x3 = MX.sym('x3');
x4 = MX.sym('x4');
x  = [x1; x2; x3; x4];    
u1  = MX.sym('u1'); % Controls
u2  = MX.sym('u2'); % Controls
u = [u1;u2];
% Update system position
M = get_MassMatrix([x1;x2]);
C = get_CoriolisVector([x1;x2],[x3;x4]);

ode = [[x3;x4]; inv(M)*(-C + [u1;u2])];
% ode = [[x3;x4]; inv(M)*(-C*[x3;x4] -G - vp1*[x3;x4] - vp2*[sign(x3);sign(x4)] + [u1;u2])];

f = Function('f',{x,u},{ode},{'x','u'},{'ode'});
intg_options = struct;
intg_options.tf = ti;
intg_options.simplify = true;

dae = struct;
dae.x = x;         % What are states?
dae.p = u;         % What are parameters (=fixed during the integration horizon)?
dae.ode = f(x,u);  % Expression for the right-hand side

intg = integrator('intzg','rk',dae,intg_options);
res = intg('x0',x,'p',u); % Evaluate with symbols
x_next = res.xf;
F = Function('F',{x,u},{x_next},{'x','u'},{'x_next'});
opti = casadi.Opti();

while (j <= Iterations)
    SSQfun = Polyhedron([SS', Qfun']);
    SS = SSQfun.V(:,1:4)';
    Qfun = SSQfun.V(:, end)';
    
    t = 1;        % Initialize time
    x_LMPC = x0;  % Initial condition
  
    tollerance = 10^(-3);
    exitFlag = 0;
    sam_t = 1;
    a = 1;
    ext_tq(:,1) = [0.1; 0];
    while ( exitFlag == 0 )
        clc
        fprintf('Time step: %d, Iteration: %d, Cost: %13.15f\n', [t, j, IterationCost_out{j}(1)]);
        if t>=2
            (x_LMPC(:,t)-ref_q)'*(x_LMPC(:,t)-ref_q)
        end
        
        % Solve the LMPC at time t of the j-th iteration
        tic;
        [~, uPred] = FTOCP(x_LMPC(:,t), N, Q, R, Qfun, SS, LMPC_options,ti,ref_q);

        M = get_MassMatrix(x_LMPC(1:2,t));
        C = get_CoriolisVector(x_LMPC(1:2,t),x_LMPC(3:4,t));
        
        u_LMPC(:,t) =  uPred;
        ext_tq(:,t+1) = rk5(ext_tq(:,t),t*ti,ti);
        
        x_LMPC(:,t+1) = full(F(x_LMPC(:,t),u_LMPC(:,t) + ext_tq(:,t)));
        t = t + 1;
        Next_j = j;
        % Check exits conditions
        
        if t >= (30/ti)+1
            exitFlag = 1;
            j = Next_j;
       end

    end
    
    % Now save the data, update cost and safe set.
    x_cl_out{j+1} = x_LMPC;
    u_cl_out{j+1} = u_LMPC;
    IterationCost_out{j+1} = ComputeCost(x_LMPC-ref_q, u_LMPC, Q, R);
    
    SS   = [SS, x_LMPC];
    Qfun = [Qfun, IterationCost_out{j+1}];
    
    % increase Iteration index and restart
    j = j + 1;
    if j <= Iterations
        clear x_LMPC
        clear u_LMPC
        clear ext_tq
    end
    
end

function out = sat(u)
global umax
if abs(u) <= umax
out = u;
else
out = umax*sign(u);
end
end
end