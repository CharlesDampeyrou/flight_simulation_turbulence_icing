% Script to display a single airspeed control test

import controlOptim.airspeed_control_test
import controlOptim.airspeed_control_loss

airspeed_kp = 0.0531;
airspeed_ki = 0.0420;
airspeed_kd = 0;
airspeed_pid_params = [airspeed_kp, airspeed_ki, airspeed_kd];

initial_airspeed = 240;
initial_altitude = 5000;
initial_mass = 3400; %Empty mass : 2790kg, MTOW : 5669kg

target_airspeed = 200;

flaps=false;
turb=true;

airspeed_control_test(airspeed_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_airspeed, flaps, turb, true);
% altitude_control_loss(altitude_pid_params)