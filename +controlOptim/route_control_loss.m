function loss = route_control_loss(route_pid_params)
% This function computes a loss for a given set of route angle control
% parameters. The loss is computed as the sum of the losses for different initial mass and airspeed
import controlOptim.controler_loss
import controlOptim.route_control_test

initial_airspeed1 = 60; %m/s
initial_airspeed2 = 230; %m/s

initial_altitude = 5000;
initial_mass1 = 4700; %Empty mass : 2790kg, MTOW : 5669kg
initial_mass2 = 3000; %Empty mass : 2790kg, MTOW : 5669kg

target_route_angle = 10*pi/120;

t_end=119;

target_route_hist = [0*ones(1/0.05, 1); target_route_angle*ones((t_end-1)/0.05+1, 1)];


t_idx_change = 1/0.05+1;
overshooting_coef=10; % loss=10 for 100% overshoot
rising_coef=10/(10*20); % loss=10 for 10s to reach 63% of the command
error_integration_coef=10/(20*t_end*1*pi/180); % loss=10 for a constant error of 1°
command_speed_coef=10/(5*pi/180/20); % loss=10 for a maximum target speed of phi of 5°/s
command_oscillation_coef=100; % loss=100 if the command ranges 3 times the
% difference between its max and min

loss = 0;
for airspeed=[initial_airspeed1, initial_airspeed2]
for mass=[initial_mass1, initial_mass2]
    if airspeed<75
        flaps=true;
    else
        flaps=false;
    end
    [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = route_control_test(route_pid_params, airspeed, ...
        initial_altitude, mass, target_route_angle, flaps, true, false);
    loss = loss + controler_loss(target_route_hist, intermediary_terms_hist(:,2), x_hist(:,12), t_idx_change, overshooting_coef, rising_coef, error_integration_coef, command_speed_coef, command_oscillation_coef);
end
end
end