function loss = overshooting_loss(target, u, y, t_idx_change)
% Computes a score on the overshooting to evaluate the response to a
% controler. The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timestamp
%   - u : command
%   - y : the response obtained by the system
%   - t_change : the instant at which the target changes
% The score is defined as the difference between the maximum value reached
% and the second target value, divided by the size of the echelon.
if target(end) < target(1) % To handle a negative echelon
    target = -target;
    y = -y;
end
overshoot = max(0, max(y(t_idx_change:end))-y(end));
loss = overshoot/(target(end)-target(1));
end

