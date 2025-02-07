function [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = ...
    route_control_test(rudder_pid_params, initial_airspeed, ...
    initial_altitude, initial_mass, target_route, flap_instruc, use_turb, plotplotplot)
    % Function to test the drift control parameters using a Heaviside
    % function. The plane starts with 1s at a target route of 0 then 29s with a target route of target_route
    % Inputs:
    %   rudder_pid_params: 1x3 array with the PID parameters for the drift controler
    %   initial_airspeed: initial airspeed of the plane in m.s^-1
    %   initial_altitude: initial altitude of the plane in m
    %   initial_mass: initial mass of the plane in kg
    %   target_route: target route angle of the plane in rad
    %   flap_instruc: initial flaps position in rad
    %   use_turb: boolean to use turbulence or not
    %   plotplotplot: boolean to plot the results or not
    % Outputs:
    %   x_hist: nx12 array with the state of the plane at each time step
    %   u_hist: nx8 array with the control inputs of the plane at each time step
    %   intermediary_terms_hist: nx8 array with the intermediary terms of the controler at each time step
    %   usefullvar_list: nx1 cell array with the useful variables of the simulation at each time step
    
    import control.planeControler
    import controlOptim.compute_test_trajectory
    import controlOptim.show_flight_plots

    % Initialisation of the aircraft state and controls
    initial_heading = 0;
    initial_flaps = flap_instruc;
    initial_icing = 0;
    initial_gear = 0;
    [x_init, u_init] = get_initial_state(initial_airspeed, initial_altitude, ...
        initial_heading, initial_mass, initial_flaps, initial_icing, ...
        initial_gear);

    % Creation of the controler
    delta_t = 0.05;

    throttle_initial_value = u_init(4);

    throttle_kp = 0.0531;
    throttle_ki = 0.0420;
    throttle_kd = 0;
    throttle_controler_params = [throttle_kp, throttle_ki, throttle_kd, ...
        delta_t, throttle_initial_value];

    flaps_airspeed_lim = 75; %m.s^-1
    flaps_altitude_lim = 500; %m
    flaps_controler_params = [flaps_airspeed_lim, flaps_altitude_lim];

    stabilator_pid_ki = 0.1;
    stabilator_initial_value = u_init(7);
    stabilator_min_value = -7*pi/180;
    stabilator_max_value = 0.4*pi/180;
    stabilator_controler_params = [stabilator_pid_ki, delta_t, ...
        stabilator_initial_value, stabilator_min_value, ...
        stabilator_max_value];

    gear_altitude_lim = 350; %m
    gear_controler_params = [gear_altitude_lim];

    max_altitude_error = 500; %m
    gamma_pid_kp = 0.0004686;
    gamma_pid_ki = 0;
    gamma_pid_kd = 0.0001565;
    initial_gamma = 0;
    min_gamma = -20*pi/180;
    max_gamma = 20*pi/180;
    max_aoa_target = 15*pi/180;
    elevator_kp = 1845.3; % from gamma controler optimization
    elevator_ki = 61.1;
    elevator_kd = 20.26;
    elevator_min = -15*pi/180;
    elevator_max = -elevator_min;
    altitude_controler_params = [max_altitude_error, gamma_pid_kp, gamma_pid_ki, gamma_pid_kd, delta_t, initial_gamma, min_gamma, max_gamma, max_aoa_target, elevator_kp, elevator_ki, elevator_kd, elevator_min, elevator_max, initial_airspeed];

    max_route_error = 45*pi/180;
    roll_angle_pid_kp = 2.7059;
    roll_angle_pid_ki = 0;
    roll_angle_pid_kd = 0.0939;
    max_roll_angle = 30*pi/180;
    aileron_pid_kp = 3296;
    aileron_pid_ki = 0;
    aileron_pid_kd = 4172;
    max_aileron_angle = 20*pi/180;
    route_controler_params=[max_route_error, roll_angle_pid_kp, ...
        roll_angle_pid_ki, roll_angle_pid_kd, delta_t, max_roll_angle, ...
        aileron_pid_kp, aileron_pid_ki, aileron_pid_kd, max_aileron_angle];

    rudder_kp = rudder_pid_params(1);
    rudder_ki = rudder_pid_params(2);
    rudder_kd = rudder_pid_params(3);
    max_rudder_angle = 30*pi/180;
    rudder_controler_params=[rudder_kp, rudder_ki, rudder_kd ,delta_t, max_rudder_angle];

    controler = planeControler(altitude_controler_params, ...
        route_controler_params, rudder_controler_params, ...
        throttle_controler_params, flaps_controler_params, ...
        stabilator_controler_params, gear_controler_params, x_init, delta_t);

    % Simulation
    t_end =30;
    t_change = 1;
    if use_turb
        turbulenceFilePath = '~/Documents/data/Safire_meghatropique/turbulence_files/MIL-STD-1797A/no_turbulence/40Hz/small_turbfile.csv';
        [~, turb_data] = lireFichierCSV(turbulenceFilePath);
        turb_hist = turb_data(1:t_end*40+1,:);
    else
        turb_hist = [0,0,0,0,0,0,0,0;t_end+1,0,0,0,0,0,0,0];
    end
    instruction_hist = [
        0, 0, initial_airspeed, initial_altitude, 0
        t_change-0.001, 0, initial_airspeed, initial_altitude, 0
        t_change, target_route, initial_airspeed, initial_altitude, 0
        t_end, target_route, initial_airspeed, initial_altitude, 0
    ];
    [x_hist, u_hist, intermediary_terms_hist, usefullvar_list] = ...
        compute_test_trajectory(x_init, u_init, controler, instruction_hist, ...
        turb_hist, delta_t);

    if plotplotplot
        show_flight_plots(x_hist, u_hist, intermediary_terms_hist, ...
        usefullvar_list, delta_t, 0);
    end
end

