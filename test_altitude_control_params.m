% Script to display a single altitude control test

import controlOptim.altitude_control_test
import controlOptim.altitude_control_loss

gamma_kp = 0.0004686;
gamma_ki = 0;
gamma_kd = 0.0001565;
altitude_pid_params = [gamma_kp, gamma_ki, gamma_kd];

initial_airspeed = 230; %m/s
initial_altitude = 5000;
initial_mass = 3000; %Empty mass : 2790kg, MTOW : 5669kg

target_altitude = 5050;

flaps=false;
turbulence=true;

altitude_control_test(altitude_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_altitude, flaps, turbulence, true);
%altitude_control_loss(altitude_pid_params)