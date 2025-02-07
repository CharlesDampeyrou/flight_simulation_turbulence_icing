function loss = roll_control_loss(roll_pid_params)
% This function computes a loss for a given set of roll control
% parameters. The loss is computed as the sum of the losses for different initial mass and airspeed
import controlOptim.controler_loss
import controlOptim.roll_control_test

initial_airspeed1 = 60; %m/s
initial_airspeed3 = 230; %m/s

initial_altitude = 5000;
initial_mass1 = 4700; %Empty mass : 2790kg, MTOW : 5669kg
initial_mass2 = 3000; %Empty mass : 2790kg, MTOW : 5669kg
target_roll_angle1 = 10*pi/180;

t_idx_change = 1/0.05+1;
overshooting_coef=10; % loss=10 for 100% overshoot
rising_coef=0.5; % loss=10 for 1s to reach 63% of the command
error_integration_coef=0.287; % loss = 10 for a constant error of 5°
command_speed_coef=1146; % loss=10 for a maximum speed of the ailerons of 10°/s
command_oscillation_coef=20; % loss=20 if the command ranges 2 times the
% difference between its max and min

loss=0;
for airspeed=[initial_airspeed1, initial_airspeed3]
for mass=[initial_mass1, initial_mass2]
    if airspeed<75
        flaps=true;
    else
        flaps=false;
    end
    [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = roll_control_test(roll_pid_params, airspeed, initial_altitude, mass, target_roll_angle1, flaps, true, false);
    loss = loss + controler_loss(intermediary_terms_hist(:,3), u_hist(:,2),  x_hist(2:end,10), t_idx_change, overshooting_coef, rising_coef, error_integration_coef, command_speed_coef, command_oscillation_coef);
end
end
end
