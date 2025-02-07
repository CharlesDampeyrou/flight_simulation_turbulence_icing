function loss = controler_loss(target, u, y, t_idx_change,...
    overshooting_coef, rising_coef, error_integration_coef, command_speed_coef, ...
    command_oscillation_coef)
% Computes a loss to evaluate the response of a controler to an echelon
% input. The target has to be a constant value from 0 to t_idx_change
% then a new taget value till the end of the arrays (echelon test).
% Inputs :
%   - target : the target at each timesteps
%   - y : the response obtained by the system at the same timesteps
%   - t_change : the instant at which the target changes
% The loss is defined using the overshooting_loss, rise_time_loss
% error_integration_loss, command_speed_loss and command_oscillation_loss
import controlOptim.overshooting_loss
import controlOptim.rise_time_loss
import controlOptim.error_integration_loss
import controlOptim.command_speed_loss
import controlOptim.command_oscillation_loss
os_loss = overshooting_loss(target, u, y, t_idx_change) * overshooting_coef;
r_loss = rise_time_loss(target, u, y, t_idx_change) * rising_coef;
ei_loss = error_integration_loss(target, u, y, t_idx_change) * error_integration_coef;
cs_loss = command_speed_loss(target, u, y, t_idx_change) * command_speed_coef;
co_loss = command_oscillation_loss(target, u, y, t_idx_change) * command_oscillation_coef;
loss = os_loss + r_loss + ei_loss + cs_loss + co_loss;
end

