function [ x_cl, u_cl ] = solve_CFTOCP(xe, leader, N, Q, R,ti,u_h)
import casadi.*
x1 = MX.sym('x1'); % States
x2 = MX.sym('x2');
x3 = MX.sym('x3');

x  = [x1; x2; x3];    
u1  = MX.sym('u1'); % Controls
u2  = MX.sym('u2'); % Controls

vl  = MX.sym('vl'); % Leader Velocity
wl  = MX.sym('wl'); % Leader Ang Velocity
u = [u1; u2; vl; wl];


ode = [u2*x2+vl*cos(x3)-u1;-u2*x1+vl*sin(x3);wl-u2];

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

% Total n
tn_x = round((N)/ti);
tn_u = round((N)/u_h);
num_time_u = round((u_h)/ti);

x = opti.variable(3,tn_x+1); % Decision variables for state trajetcory, tn + next_state
u = opti.variable(2,tn_u+1);
p = opti.parameter(3,1);  % Parameter (not optimized over)
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
Cost = Cost + x(:,tn_x+1)'*Q*x(:,tn_x+1);
opti.minimize(Cost);

% Select state and input constraints matrices from polyhedrons
max_s = 3;
max_u = 8;

leader_vel(:,1) = leader;
t = 0;
a = 1;
for i=1:tn_x
    if (i == 1)
        opti.subject_to(-max_u <= u(1,a) <= max_u);
        opti.subject_to(-max_u <= u(2,a) <= max_u);
        sam_u = u(1:2,a);
        a = a + 1;
    elseif (t == num_time_u) 
        opti.subject_to(-max_u <= u(1,a) <= max_u);
        opti.subject_to(-max_u <= u(2,a) <= max_u);
        sam_u = u(1:2,a);
        a = a + 1;
        t = 0;
    end
    t0 = (i-1)*ti;
    t1 = (i-1)*ti+ti;
    t_interval = [t0, t1];

    leader_vel(:,i+1) = Leader_vel_rk(leader_vel(:,i),t_interval,ti);
    opti.subject_to(x(:,i+1)==F(x(:,i),[sam_u;leader_vel(1,i);leader_vel(2,i)]));
    t = t + 1;
end

% System Dynamics
for i = 1:tn_x
    opti.subject_to(-max_s <= x(1,i) <= max_s);
    opti.subject_to(-max_s <= x(2,i) <= max_s);
    opti.subject_to(-max_s <= x(3,i) <= max_s);

end

opti.subject_to(x(:,1)==p);
opti.subject_to(x(:,tn_x+1)==[0;0;0]);

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

