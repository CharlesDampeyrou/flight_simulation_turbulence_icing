function loss = altitude_control_loss(altitude_pid_params)
% This function computes a loss for a given set of altitude control
% parameters. The loss is computed as the sum of the losses for d different initial mass and airspeed
import controlOptim.controler_loss
import controlOptim.altitude_control_test

initial_airspeed1 = 60; %m/s
initial_airspeed2 = 230; %m/s

initial_altitude = 5000;
initial_mass1 = 3000; %Empty mass : 2790kg, MTOW : 5669kg
initial_mass2 = 4700; %Empty mass : 2790kg, MTOW : 5669kg
target_altitude = 5030;

t_end=119;
target_altitude_hist = [5000*ones(1/0.05, 1); target_altitude*ones((t_end-1)/0.05+1, 1)];

t_idx_change = 1/0.05+1;
overshooting_coef=10; % loss=10 for 100% overshoot
rising_coef=0.1/(2.5*20); % loss=0.1 for 2.5s to reach 63% of the command
error_integration_coef=10/(20*t_end*1); % loss=10 for a constant error of 1m
command_speed_coef=10/(5*pi/180/20); % loss=10 for a maximum command speed of the flight path of 5°/s
command_oscillation_coef=10; % loss=10 if the command ranges 3 times the
% difference between its max and min

loss=0;
for airspeed=[initial_airspeed1, initial_airspeed2]
for mass=[initial_mass1, initial_mass2]
    if airspeed<75
        flaps=true;
    else
        flaps=false;
    end
    [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = altitude_control_test(...
        altitude_pid_params, airspeed, ...
        initial_altitude, mass, target_altitude, flaps, true, false);
    loss = loss + controler_loss(target_altitude_hist, intermediary_terms_hist(:, 1),  -x_hist(:,6),t_idx_change, overshooting_coef, rising_coef, error_integration_coef, command_speed_coef, command_oscillation_coef);
end
end
end

