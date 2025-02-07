function loss = command_speed_loss(target, u, y, t_idx_change)
% Computes a score on the command changing speed for a controler.
% The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timestamp
%   - u : command
%   - y : the response obtained by the system
%   - t_change : the instant at which the target changes
% The score is defined as the maximum absolute difference between the
% command over 2 timesteps after the echelon. The timestep of the echelon
% is not taken into account since the stiffness of the target create a fast
% variation of the command input.
abs_diff = abs(u(t_idx_change+2:end)-u(t_idx_change+1:end-1));
loss = max(abs_diff);
end

