function [turb] = getTurbulence(t, turbData)
% Returns the turbulence interpolated at time t
% Inputs :
%   - t : time at which turbulence is required
%   - turbData : matrix of turbulence data. Columns : time, presence of turbulence (boolean), u, v, w, p, q, r
% Outputs :
%   - turb : turbulence at time t : [presence of turbulence, u, v, w, p, q, r]
turb = interp1qr(turbData(:,1), turbData(:, 2:8), t)';
end

