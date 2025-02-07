function loss = settling_time_loss(target, u, y, t_idx_change)
% Computes a score on the settling time to evaluate the response to a
% controler. The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timestamp
%   - u : command
%   - y : the response obtained by the system
%   - t_change : the instant at which the target changes
% The score is defined as the number of timesteps the system takes to
% settle at 5% or less of the target. If the start of the echelon is not 0,
% the error between the target and the system's response has to be less
% than 5% of the difference between the first and second value of the
% echelon.
% If the response is not settled at 5% at the end of the simulation, the
% settling time at 25% is added to the score.
if target(end) < target(1) % To handle a negative echelon
    target = -target;
    y = -y;
end
v_min1 = target(1) + 0.95*(target(end)-target(1));
v_min2 = target(1) + 0.75*(target(end)-target(1));
v_max1 = target(1) + 1.05*(target(end)-target(1));
v_max2 = target(1) + 1.25*(target(end)-target(1));
loss = find(y(t_idx_change:end)>v_max1 | y(t_idx_change:end)<v_min1, 1, 'last')+1;
if loss>length(y)-t_idx_change % response is not settled at 5% at the end of the simulation
    % Adding the settling time at 20%
    loss = loss + find(y(t_idx_change:end)>v_max2 | y(t_idx_change:end)<v_min2, 1, 'last')+1;
end
end

