function cqdq = get_CoriolisVector(q,dq)
global a b c
cqdq = [-b*dq(2)^2*sin(q(2)-q(1));
        b*dq(1)^2*sin(q(2)-q(1))];
end