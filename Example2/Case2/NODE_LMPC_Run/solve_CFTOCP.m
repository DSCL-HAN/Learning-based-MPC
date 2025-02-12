function [ x_cl, u_cl ] = solve_CFTOCP(xe, qref, N, Q, R,ti,u_h)
import casadi.*

x1 = MX.sym('x1'); % States
x2 = MX.sym('x2');
x3 = MX.sym('x3');
x4 = MX.sym('x4');

x  = [x1; x2; x3; x4];    
u1  = MX.sym('u1'); % Controls
u2  = MX.sym('u2'); % Controls

Fe1  = MX.sym('Fe1'); % External Force 1
Fe2  = MX.sym('Fe2'); % " 2
u = [u1; u2; Fe1; Fe2];
% u = [u1; u2];

a = 5.6794;
b = 1.4730;
c = 1.7985;

M = [a, b*cos(x2-x1);
    b*cos(x2-x1), c];
C = [-b*x4^2*sin(x2-x1);
    b*x3^2*sin(x2-x1)];

ode = [x3;x4;inv(M)*(-C+[u1;u2]+[Fe1;Fe2])];

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
tn_x = round((N)/ti);
tn_u = round((N)/u_h);
num_time_u = round((u_h)/ti);

x = opti.variable(4,tn_x+1); % Decision variables for state trajetcory, tn + next_state
u = opti.variable(2,tn_u);
p = opti.parameter(4,1);  % Parameter (not optimized over)
t = 0;
Cost=0;
a = 1;

for i=1:tn_x
    if (i == 1)
        sam_u = u(:,a);
        a = a + 1;
    elseif (t == num_time_u) 
        sam_u = u(:,a);
        a = a + 1;
        t = 0;
    end
    
    Cost = Cost + x(:,i)'*Q*x(:,i) + sam_u'*R*sam_u; 

    t = t + 1;
        
end

Cost = Cost + (x(:,tn_x+1)')*Q*(x(:,tn_x+1));
opti.minimize(Cost);

t = 0;
a = 1;
% max_u = 5;
NODE_qr(:,1) = qref;
for i=1:tn_x
    if (i == 1)
        sam_u = u(1:2,a);
        % opti.subject_to(-max_u <= u(1,a) <= max_u);
        % opti.subject_to(-max_u <= u(2,a) <= max_u);
        a = a + 1;
    elseif (t == num_time_u) 
        sam_u = u(1:2,a);
        % opti.subject_to(-max_u <= u(1,a) <= max_u);
        % opti.subject_to(-max_u <= u(2,a) <= max_u);
        a = a + 1;
        t = 0;
    end
    NODE_qr(:,i+1) = rk5(NODE_qr(:,i),ti*(i-1),ti);
    opti.subject_to(x(:,i+1)==F(x(:,i),[sam_u;NODE_qr(1,i);NODE_qr(2,i)]));

    t = t + 1;
end

% State Constraint
% max_s = 1;
% 
% for i = 1:tn
%     opti.subject_to(-max_s <= x(1,i) <= max_s);
%     opti.subject_to(-max_s <= x(2,i) <= max_s);
%     opti.subject_to(-max_s <= x(3,i) <= max_s);
%     opti.subject_to(-max_s <= x(4,i) <= max_s);
% end
P = eye(4);
r = 0.001;
opti.subject_to(x(:,1)==p);
opti.subject_to(x(:,tn_x+1)'*P*x(:,tn_x+1)<=r);

opti.set_value(p,xe);
opti.solver('ipopt');
sol = opti.solve();


x = sol.value(x);
u = sol.value(u);
x_cl = [];
for i = 1:(tn_x+1)
    x_cl = [x_cl, double(x(:,i))];
end

u_cl = [];
for i = 1:tn_u
    u_cl = [u_cl, double(u(:,i))];
end

end

