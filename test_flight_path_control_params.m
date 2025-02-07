% Script to display a single test of the flight path control

import controlOptim.gamma_control_test
import controlOptim.gamma_control_loss

elevator_pid_kp = 1845.3;
elevator_pid_ki = 61.1;
elevator_pid_kd = 20.26;
flight_path_pid_params = [elevator_pid_kp, elevator_pid_ki, elevator_pid_kd];

initial_airspeed=230;
initial_altitude = 5000;
initial_mass = 4500; %Empty mass : 2790kg, MTOW : 5669kg
target_flight_path = 5*pi/180; % in rad

flaps = false;
turbulence = true;

gamma_control_test(flight_path_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_flight_path, flaps, turbulence, true);
%loss = gamma_control_loss(flight_path_pid_params);