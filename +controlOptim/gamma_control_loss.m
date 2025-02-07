function loss = gamma_control_loss(flight_path_pid_params)
% This function computes a loss for a given set of flight path control
% parameters. The loss is computed as the sum of the losses for différent initial mass and airspeed
import controlOptim.controler_loss
import controlOptim.gamma_control_test

initial_airspeed1 = 65; %m/s
initial_airspeed2 = 230; %m/s

initial_altitude = 5000;
initial_mass1 = 3000; %Empty mass : 2790kg, MTOW : 5669kg
initial_mass2 = 4700; %Empty mass : 2790kg, MTOW : 5669kg, 1700kg of fuel is enough for the simulation time

target_gamma = 5*pi/180;

t_idx_change = 1/0.05+1;
t_end = 119; % Change according to the simulation time in gamma_control_test.m
overshooting_coef=10; % loss=10 for 100% overshoot
rising_coef=1/(2.5*20); % loss=1 for 2.5s to reach 63% of the command
error_integration_coef=50/(20*t_end*1*pi/180); % loss=10 for a constant error of 1°
command_speed_coef=10/(30*pi/180/20); % loss=10 for a maximum command speed of the elevator of 30°/s
command_oscillation_coef=20; % loss=20 if the command ranges 3 times the
% difference between its max and min

loss = 0;
for initial_airspeed=[initial_airspeed1, initial_airspeed2]
for initial_mass=[initial_mass1, initial_mass2]
    if initial_airspeed<75
        flaps=1;
    else
        flaps=0;
    end
    [~, u_hist, intermediary_terms_hist, usefullvar_list] = gamma_control_test(...
    flight_path_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_gamma, flaps, true, false);
    loss = loss + controler_loss(intermediary_terms_hist(:, 1), u_hist(:, 1),  usefullvar_list(:,15), t_idx_change, overshooting_coef, rising_coef, error_integration_coef, command_speed_coef, command_oscillation_coef);
end
end
end

