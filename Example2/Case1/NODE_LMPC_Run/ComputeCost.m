function [ Qfun] = ComputeCost(x,u,Q,R)
for i = 1:(size(x,2))
    Index = size(x,2)-i+1; % Need to start from the end
    if i == 1
        Cost(Index) = x(:,Index)'*Q*x(:,Index);    
    else
        Cost(Index) = Cost(Index+1) + x(:,Index)'*Q*x(:,Index) + u(:,Index)'*R*u(:,Index);    
    end
Qfun = Cost;
end

