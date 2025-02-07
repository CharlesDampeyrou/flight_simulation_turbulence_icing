% Script to display a single test of the drift control

import controlOptim.rudder_control_test
import controlOptim.rudder_control_loss

rudder_kp = 155190;
rudder_ki = 101135;
rudder_kd = 1639;
rudder_pid_params = [rudder_kp, rudder_ki, rudder_kd];

initial_airspeed = 230; %m/s
initial_altitude = 5000;
initial_mass = 3000; %Empty mass : 2790kg, MTOW : 5669kg

target_route = 15*pi/180;

flaps=false;
turb=true;

rudder_control_test(rudder_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_route, flaps, turb, true);
%rudder_control_loss(rudder_pid_params)