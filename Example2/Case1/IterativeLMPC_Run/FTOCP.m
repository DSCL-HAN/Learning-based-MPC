function [ x, uPred] = FTOCP( x_t , N, Q, R, Qfun, SS, LMPC_options,ti,ref_q)
% FTOCP solves the Finite Time Optimal Control Problem
% The function takes as inputs
% - x_t: state of the system at time t
% - N: horizon length
% - (Q,R): matrices defining the running cost
% - Qfun: vector collecting the cost-to-go up to the current iteration
% - SS: matrix collecting the stored states up to the current iteration
% - (A,B): matrices defining the system dynamics
% - X: polyhedron representng the state constraints
% - U: polyhedron representng the input constraints
% - Solver: solver to use for the FTOCP

% Define Yalmip Variables
import casadi.*
x1 = MX.sym('x1'); % States
x2 = MX.sym('x2');
x3 = MX.sym('x3');
x4 = MX.sym('x4');

x  = [x1; x2; x3; x4];    
u1  = MX.sym('u1'); % Controls
u2  = MX.sym('u2'); % Controls

u = [u1; u2];

M = get_MassMatrix([x1;x2]);
C = get_CoriolisVector([x1;x2],[x3;x4]);

ode = [[x3;x4]; inv(M)*(-C + [u1;u2])];

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

x = opti.variable(4,N+1); % Decision variables for state trajetcory
u = opti.variable(2,N);
p = opti.parameter(4,1);  % Parameter (not optimized over)
lambda = opti.variable(length(Qfun),1);

% ======= Constraints Definition ======

% Initial Condition
opti.subject_to(x(:,1)==p);
opti.subject_to((x(:,N+1))==SS*lambda);
opti.subject_to(lambda >= 0);
opti.subject_to(ones(1,length(Qfun))*lambda == 1);
Cost=0;

for k=1:N
        M = get_MassMatrix(x(1:2,k));
        C = get_CoriolisVector(x(1:2,k),x(3:4,k));

        opti.subject_to(x(:,k+1)==F(x(:,k),u(:,k)));
        Cost = Cost + (x(1:4,k)-ref_q)'*Q*(x(1:4,k)-ref_q) + u(:,k)'*R*u(:,k);


end
Cost = Cost + (x(1:4,N+1)-ref_q)'*Q*(x(1:4,N+1)-ref_q);
Cost = Cost + Qfun*lambda;

max_s = 1;
max_u = 2;
for k = 1:N
    opti.subject_to(-max_s <= x(1,k) <= max_s);
    opti.subject_to(-max_s <= x(2,k) <= max_s);
    opti.subject_to(-max_s <= x(3,k) <= max_s);
    opti.subject_to(-max_s <= x(4,k) <= max_s);

    opti.subject_to(-max_u <= u(1,k) <= max_u);
    opti.subject_to(-max_u <= u(2,k) <= max_u);
    if k == N
        opti.subject_to(-max_s <= x(1,k+1) <= max_s)
        opti.subject_to(-max_s <= x(2,k+1) <= max_s);
        opti.subject_to(-max_s <= x(3,k+1) <= max_s);
        opti.subject_to(-max_s <= x(4,k+1) <= max_s);
    end
end

opti.minimize(Cost);
opti.set_value(p,x_t);
opti.solver('ipopt');
sol = opti.solve();

x = sol.value(x);
u = sol.value(u);

uPred = double(u(:,1));

end

