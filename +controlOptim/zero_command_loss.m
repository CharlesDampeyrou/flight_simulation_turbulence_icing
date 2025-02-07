function loss = zero_command_loss(u, y, t_idx_change, integration_coef, command_speed_coef, command_oscillation_coef)
    % Function to compute the loss of a controler when the target is constantly 0. The loss is the sum of :
    % - a loss on the error integration
    % - a loss on the command speed
    % - a loss on the command oscillation
    import controlOptim.error_integration_loss
    import controlOptim.command_speed_loss
    import controlOptim.command_oscillation_loss
    target = zeros(size(y));
    loss1 = error_integration_loss(target, u, y, t_idx_change) * integration_coef;
    loss2 = command_speed_loss(target, u, y, t_idx_change) * command_speed_coef;
    loss3 = command_oscillation_loss(target, u, y, t_idx_change) * command_oscillation_coef;
    loss = loss1 + loss2 + loss3;
end