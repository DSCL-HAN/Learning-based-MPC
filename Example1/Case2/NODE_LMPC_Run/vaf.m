function v = vaf(y,y_est)

if nargin < 2
    error('VAF requires two input arguments.');
end

if size(y,2) > size(y,1)
    y = y';
end

if size(y_est,2) > size(y_est,1)
    y_est = y_est';
end
N = size(y,1);
if size(y_est,1) ~= N
    error('Both signals should have an equal number of samples.');
end

if size(y,2) ~= size(y_est,2)
    error('Both signals should have an equal number of components.');
end
v = max(diag(100*(eye(size(y,2))-cov(y-y_est)./cov(y))),0);
end