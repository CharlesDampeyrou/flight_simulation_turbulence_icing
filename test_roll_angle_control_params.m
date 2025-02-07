% Script to display a single test of the roll angle control

import controlOptim.roll_control_test
import controlOptim.roll_control_loss

roll_kp = 3296.1;
roll_ki = 0;
roll_kd = 4172;
roll_pid_params = [roll_kp, roll_ki, roll_kd];

initial_airspeed = 230; %m/s
initial_altitude = 5000;
initial_mass = 3000; %Empty mass : 2790kg, MTOW : 5669kg

target_roll_angle = 10*pi/180;

flaps = false;
turb = true;

roll_control_test(roll_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_roll_angle, flaps, turb, true);
%roll_control_loss(roll_pid_params)