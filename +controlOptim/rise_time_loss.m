function loss = rise_time_loss(target, u, y, t_idx_change)
% Computes a score on the rise time to evaluate the response of a
% controler. The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timestamp
%   - u : command
%   - y : the response obtained by the system
%   - t_change : the instant at which the target changes
% The score is defined as the number of timesteps between the change of
% value of the target and the first time y reaches 63% of the response
if target(end) < target(1) % To handle a negative echelon
    target = -target;
    y = -y;
end
lim = target(1) + 0.63*(target(end)-target(1));
loss = find(y(t_idx_change+1:end) >= lim, 1, 'first');
if isempty(loss) % 63% of the response is never achieved
    loss = length(y)-t_idx_change+10;
end

