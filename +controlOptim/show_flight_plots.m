function show_flight_plots(x_hist, u_hist, intermediary_terms_hist, ...
    usefullvar_list, delta_t, plotfilter)
% Displays plots concerning a flight.
% input:
%   x_hist: nx13 array with the state of the aircraft at each time step
%   u_hist: nx8 array with the control inputs of the aircraft at each time step
%   intermediary_terms_hist: nx8 array with the intermediary terms of the controler at each time step
%   usefullvar_list: nx1 cell array with the useful variables of the simulation at each time step
%   delta_t: time step of the simulation
%   plotfilter: integer to filter the plots to display
t_hist = 0:delta_t:(size(u_hist,1)*delta_t);

% jz
if ismember(plotfilter, [0, 1, 2, 3])
    figure
    plot(t_hist(1:end-1), usefullvar_list(:,18));
    xlabel("Time, s")
    ylabel("JZ")
end

% altitude
if ismember(plotfilter, [0, 1, 2, 3])
    figure
    plot(t_hist, -x_hist(:,6));
    xlabel("Time, s")
    ylabel("altitude, m")
end

% flight path
if ismember(plotfilter, [0, 2, 3])
    figure
    plot(t_hist(1:end-1), intermediary_terms_hist(:,1),...
        t_hist(1:end-1), usefullvar_list(:,15));
    xlabel("Time, s")
    ylabel("Flight path, rad")
    legend("Flight path instruction", "Observed flight path")
end

% Elevator and stabilator commands
if ismember(plotfilter, [0, 1, 2, 3])
    figure
    plot(t_hist(1:end-1), u_hist(:, 1), ...
        t_hist(1:end-1), u_hist(:, 7));
    xlabel("Time, s")
    ylabel("command, rad")
    legend("Elevator", "Stabilator")
end

% airspeed
if ismember(plotfilter, [0, 1, 2, 3])
    figure
    airspeed = sqrt(x_hist(:,1).*x_hist(:,1)+x_hist(:,2).*x_hist(:,2)+x_hist(:,3).*x_hist(:,3));
    plot(t_hist, airspeed);
    xlabel("Time, s")
    ylabel("Airspeed, m/s");
end

% throttle command
if ismember(plotfilter, [0, 1, 2, 3])
    figure
    plot(t_hist(1:end-1), u_hist(:, 4));
    xlabel("Time, s")
    ylabel("Throttle, %")
end

% roll angle
if ismember(plotfilter, [0,5])
    figure
    plot(t_hist(1:end-1), x_hist(1:end-1,10),...
        t_hist(1:end-1), intermediary_terms_hist(:,2));
    xlabel("Time, s")
    ylabel("Roll angle, rad")
    legend("Roll angle", "Roll angle instruction")
end

% aileron command
if ismember(plotfilter, [0,5])
    figure
    plot(t_hist(1:end-1), u_hist(:, 2));
    xlabel("Time, s")
    ylabel("Aileron, rad")
end

% heading
if ismember(plotfilter, [0,5])
    figure
    plot(t_hist(1:end-1), x_hist(1:end-1,12));
    xlabel("Time, s")
    ylabel("Heading angle, rad")
end

% drift angle
if ismember(plotfilter, [0,5, 6])
    figure
    plot(t_hist(1:end-1), usefullvar_list(:,14));
    xlabel("Time, s")
    ylabel("Drift angle, rad")
end

%Angle of attack
if ismember(plotfilter, [0,3,5, 6])
    figure
    plot(t_hist(1:end-1), usefullvar_list(:,13));
    xlabel("Time, s")
    ylabel("Angle of attack, rad")
end

% rudder command
if ismember(plotfilter, [0,5, 6])
    figure
    plot(t_hist(1:end-1), u_hist(:, 3));
    xlabel("Time, s")
    ylabel("Rudder, rad")
end

