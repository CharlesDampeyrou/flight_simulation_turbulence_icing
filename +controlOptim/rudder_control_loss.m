function loss = rudder_control_loss(rudder_pid_params)
% This function computes a loss for a given set of drift control
% parameters. The loss is computed as the sum of the losses for different initial mass and airspeed
import controlOptim.zero_command_loss
import controlOptim.rudder_control_test

initial_airspeed1 = 60; %m/s
initial_airspeed2 = 230; %m/s

initial_altitude = 5000;
initial_mass1 = 3000; %Empty mass : 2790kg, MTOW : 5669kg
initial_mass2 = 4700; %Empty mass : 2790kg, MTOW : 5669kg

target_route_angle = 15*pi/120;

t_end=30;

t_idx_change = 1/0.05+1;
integration_coef=10/(20*t_end*1*pi/180); % loss=10 for an integrated error of 1°.s
command_speed_coef=10/(2/pi*180/20); % loss=10 for a maximum rudder command speed of 2°/s
command_oscillation_coef=1; % loss=1 if the command ranges 3 times the
% difference between its max and min

loss = 0;
for initial_airspeed=[initial_airspeed1, initial_airspeed2]
for initial_mass=[initial_mass1, initial_mass2]
    if initial_airspeed<75
        flaps=true;
    else
        flaps=false;
    end
    [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = rudder_control_test(rudder_pid_params, initial_airspeed, ...
        initial_altitude, initial_mass, target_route_angle, flaps, true, false);
    loss = loss + zero_command_loss(u_hist(:,3), usefullvar_list(:,14), t_idx_change, integration_coef, command_speed_coef, command_oscillation_coef);
end
end
end