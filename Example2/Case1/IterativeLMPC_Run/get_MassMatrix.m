function MassMatrix = get_MassMatrix(q)
global a b c
MassMatrix = [a, b*cos(q(2)-q(1));
    b*cos(q(2)-q(1)), c];
end