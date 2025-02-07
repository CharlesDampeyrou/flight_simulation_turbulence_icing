function loss = command_oscillation_loss(target, u, y, t_idx_change)
% Computes a score on the total command variations in order to limit the
% command oscillations.
% The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timestamp
%   - u : command
%   - y : the response obtained by the system
%   - t_change : the instant at which the target changes
% The score is defined as the sum of the absolute difference between the
% command over 2 timesteps after the echelon, divided by the range of
% motion of the command (normalization) -2 (the loss stays 0 is the command
% goes to a max monotonously and returns to 0 monotonously. The time before
% the echelon (and echelon included) is not taken into account.
abs_diff = abs(u(t_idx_change+2:end)-u(t_idx_change+1:end-1));
loss = max(sum(abs_diff)/(max(u)-min(u)) - 1, 0);
end

