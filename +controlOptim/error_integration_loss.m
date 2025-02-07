function loss = error_integration_loss(target, u, y, t_idx_change)
% Computes the sum of the absolute errors between the target and the output from the changing command time index
    abs_error = abs(target - y);
    loss = sum(abs_error(t_idx_change:end));
end