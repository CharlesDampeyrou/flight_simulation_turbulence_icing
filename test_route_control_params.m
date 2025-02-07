% Script to display a single test of the route control

import controlOptim.route_control_test
import controlOptim.route_control_loss

route_kp = 2.7059;
route_ki = 0;
route_kd = 0.0939;
route_pid_params = [route_kp, route_ki, route_kd];

initial_airspeed = 230; %m/s
initial_altitude = 5000;
initial_mass = 3000; %Empty mass : 2790kg, MTOW : 5669kg

target_route = 10*pi/180;
flaps = false;
turb = true;

route_control_test(route_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_route, flaps, turb, true);
% route_control_loss(altitude_pid_params)