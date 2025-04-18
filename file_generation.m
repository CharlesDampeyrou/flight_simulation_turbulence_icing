%   Copyright 2024 by Charles Dampeyrou

% This file is used to generate flight simulations using Robert F. Stengel's simulator. The main steps of the program are:
% 1. Retrieve the instructions, turbulence and icing data from .csv files
% 2. Create the initial state of the aircraft
% 3. Define the controler parameters
% 4. Run the simulation
% 5. Save the results in a .csv file
% The simulations can be computed in parallel to speed up the process.

% The variable saved in the .csv file and their definitions can be found in simulation_exploration/variable_correspondance.py


clear

disp('** 6-DOF FLIGHT Simulation **')

instructionDirPath = "~/Documents/data/Safire_meghatropique/flight_instruction_files/original";
turbulenceDirPath = "~/Documents/data/Safire_meghatropique/turbulence_files/MIL-STD-1797A/moderate/40Hz";
icingDataDirPath = "~/Documents/data/Safire_meghatropique/icing_condition_files";
simulationDirPath = "~/Documents/data/Safire_meghatropique/simulations/corrected_icing/moderate_turb_full_icing";

% Retrieve flight names and simulations already performed
instructionFiles = dir(fullfile(instructionDirPath, '*.csv'));
instructionFileNames = {instructionFiles.name};
simFiles = dir(fullfile(simulationDirPath, '*.csv'));
simNames = {simFiles.name};
for i=1:numel(simNames)
    simNames(i) = erase(simNames(i),'_simulated.csv');
end

flightNames = string(zeros(length(instructionFileNames)));
for i=1:length(instructionFiles)
    flightNames(i) = erase(instructionFileNames(i), '.csv');
end

% Creation of the parallel pool if needed
runParallel = true; % if true, change the for loop to a parfor loop
if runParallel
    numWorkers = 8;
    pool = gcp('nocreate');
    if isempty(pool)
        parpool('local', numWorkers);
    end
end

nFlights = length(flightNames);
parfor i = 1:nFlights
    % Current flight name
    flightName = flightNames(i);
    if ~any(strcmp(simNames, flightName))
    
        % Retrieving the instructions, turbulence and icing data
        instructionFilePath = fullfile(instructionDirPath, instructionFileNames{i});
        [instrucCols, instructions] = lireFichierCSV(instructionFilePath);

        turbulenceFileName = flightName + '_turbwind.csv';
        turbulenceFilePath = fullfile(turbulenceDirPath, turbulenceFileName);
        [turbCols, turbData] = lireFichierCSV(turbulenceFilePath);
        
        icingFileName = flightName + '_icing.csv';
        icingFilePath = fullfile(icingDataDirPath, icingFileName);
        [icingCols, icingData] = lireFichierCSV(icingFilePath);

        % Creation of the initial state
        initial_heading = instructions(1, 3) * pi / 180;
        initial_altitude = instructions(1, 4);
        initial_airspeed = instructions(1, 5);
        initial_mass = 4700; % Empty mass : 2790kg, MTOW : 5669kg
        initial_flaps = 38*pi/180; % Flaps initialy extended
        initial_icing = 0; % No icing at the beginning
        initial_gear = 1; % Gear extended

        [x_init, u_init] = get_initial_state(initial_airspeed, initial_altitude, initial_heading, initial_mass, initial_flaps, initial_icing, initial_gear);

        x = x_init;
        u = u_init;

        % Creation of the history variables
        ti = 0;
        tf = instructions(length(instructions), 1);
        delta_t = 0.05;
        nb_steps = (tf-ti)/delta_t;
        t_steps = ti:delta_t:tf;
        hist_x = zeros(nb_steps+1, 16);
        hist_x(1,:) = x;
        hist_u = zeros(nb_steps+1, 8);
        hist_u(1,:) = u;
        hist_intermediaryTerms = zeros(nb_steps, 2);
        hist_instruc = zeros(nb_steps, 4);
        hist_turbulence = zeros(nb_steps, 1);
        hist_icing = zeros(nb_steps, 1);
        hist_usedfullvars = zeros(nb_steps, 25);
        hist_plane_properties = zeros(nb_steps, 8);
        
        % Controler initialization
        max_altitude_error = 500; %m
        gamma_pid_kp = 0.0004686;
        gamma_pid_ki = 0.000004;
        gamma_pid_kd = 0.0001565;
        initial_gamma = 0;
        min_gamma = -20*pi/180;
        max_gamma = 20*pi/180;
        max_aoa_target = 13*pi/180;
        elevator_kp = 1845.3;
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
        route_controler_params=[max_route_error, roll_angle_pid_kp, roll_angle_pid_ki, roll_angle_pid_kd, delta_t, max_roll_angle, aileron_pid_kp, aileron_pid_ki, aileron_pid_kd, max_aileron_angle];
        
        rudder_pid_kp = 288055;
        rudder_pid_ki = 138267;
        rudder_pid_kd = 9628;
        rudder_max_value = 30*pi/180;
        rudder_controler_params = [rudder_pid_kp, rudder_pid_ki, rudder_pid_kd, delta_t, rudder_max_value];
        
        throttle_pid_kp = 0.0531;
        throttle_pid_ki = 0.0420;
        throttle_pid_kd = 0;
        throttle_initial_value = u_init(4);
        throttle_controler_params = [throttle_pid_kp, throttle_pid_ki, ...
            throttle_pid_kd, delta_t, throttle_initial_value];
        
        flaps_airspeed_lim = 75; %m.s^-1
        flaps_altitude_lim = 400; %m
        flaps_controler_params = [flaps_airspeed_lim, flaps_altitude_lim];
        
        stabilator_pid_ki = 0.01;
        stabilator_initial_value = u_init(7);
        stabilator_min_value = -7*pi/180;
        stabilator_max_value = 0.4*pi/180;
        stabilator_controler_params = [stabilator_pid_ki, delta_t, ...
            stabilator_initial_value, stabilator_min_value, ...
            stabilator_max_value];
        
        gear_altitude_lim = 250; %m
        gear_controler_params = [gear_altitude_lim];
        
        PlaneControler = control.planeControler(altitude_controler_params, ...
            route_controler_params, rudder_controler_params, ...
            throttle_controler_params, flaps_controler_params, ...
            stabilator_controler_params, gear_controler_params, x, ...
            delta_t);

        for j=1:nb_steps
            % Computations at the j-th time step
            t = t_steps(j);
            % Getting the flight plan at time t
            [target_route, target_airspeed, target_altitude, flaps_instruc] = GetInstruction(t, instructions);
            instruc = [target_route, target_airspeed, target_altitude, flaps_instruc];

            % Creating a subset of the turbulence and icing data to accelerate the computation
            turbDataIndices = (turbData(:, 1) >= t-1) & (turbData(:, 1) <= t+delta_t+1);
            turbDataSmall = turbData(turbDataIndices, :);
            icingDataIndices = (icingData(:, 1) >= t-2) & (icingData(:, 1) <= t+delta_t+2);
            icingDataSmall = icingData(icingDataIndices, :);

            % Getting usefulll intermediate variables from the simulation
            turb = getTurbulence(t, turbDataSmall);
            icing = interp1(icingDataSmall(:,1), icingDataSmall(:,2), t, 'previous');
            [CD,CL,CY,Cl,Cm,Cn,Thrust,airDens,airPres,temp,V,Mach,alphar,betar,gammar,xir,nx,ny,nz,xdot1,xdot2,xdot3,xdot7,xdot8,xdot9] = GetUsefullVars(x, u, t, turbDataSmall, icingDataSmall);

            % Updating the commands
            [u, intermediaryTerms] = PlaneControler.step(hist_x(j,:), ...
                turb, target_route, target_airspeed, target_altitude, ...
                flaps_instruc);

            % Integration of the equations of motion for delta_t
            [~, xinteg] = ode15s(@(t, x) EoM(t,x,u, turbDataSmall, icingDataSmall), [t_steps(j), t_steps(j+1)], x);
            x = xinteg(end, :)';

            % Saving the results for the j-th time step
            hist_x(j+1,:) = x';
            hist_u(j+1, :) = u;
            hist_intermediaryTerms(j, :) = intermediaryTerms;
            hist_instruc(j,:) = instruc;
            hist_usedfullvars(j,:) = [CD,CL,CY,Cl,Cm,Cn,Thrust,airDens,airPres,temp,V,Mach,alphar,betar,gammar,xir, nx,ny,nz,xdot1,xdot2,xdot3,xdot7,xdot8,xdot9];
            [Ixx, Iyy, Izz, Ixz, S, b, cBar, sfc] = GetPlaneProperties(hist_x(j, 13), Mach);
            hist_plane_properties(j, :) = [Ixx, Iyy, Izz, Ixz, S, b, cBar, sfc];
            
            if turb(1) > 1.0e-6
                hist_turbulence(j, 1) = 1;
            end
            if icing > 1.0e-6
                hist_icing(j, 1) = 1;
            end
            if rem(j, 500) == 0
                sprintf('Flight %i, %f %% of the simulation executed', i, 100*j/nb_steps)
            end
        end


        % Saving the simulation results
        time_varname = 'time';
        
        x_varname = {
            'ub'
            'vb'
            'wb'
            'xe'
            'ye'
            'ze'
            'pr'
            'qr'
            'rr'
            'phir'
            'thetar'
            'psir'
            'm'
            'fl'
            'ice'
            'ge'
        }';

        u_varname = {
            'dEr'
            'dAr'
            'dRr'
            'dT'
            'dASr'
            'dFr'
            'dSr'
            'dGe'
        }';
        
        intermediaryTerms_varname = {
            'tgammar'
            'tphir'
        }';
        
        instructions_varname = {
            'txir'
            'ttas'
            'th'
            'tfl'
        }';

        usefullvars_varname = {
            'cx'
            'cz'
            'cy'
            'cl'
            'cm'
            'cn'
            'thr'
            'rho'
            'p'
            'temp'
            'tas'
            'mach'
            'alphar'
            'betar'
            'gammar'
            'xir'
            'nx'
            'ny'
            'nz'
            'axb'
            'ayb'
            'azb'
            'aphir'
            'athetar'
            'apsir'
        }';
        
        plane_properties_varname = {
            'Ixx'
            'Iyy'
            'Izz'
            'Ixz'
            'S'
            'b'
            'cBar'
            'sfc'
        }';

        turb_varname = 'Turbulence';
        
        icing_varname = 'Icing';

        var_names = [time_varname x_varname u_varname intermediaryTerms_varname instructions_varname usefullvars_varname plane_properties_varname turb_varname icing_varname];
        to_save = [t_steps(1:end-1)' hist_x(1:end-1,:) hist_u(1:end-1,:) hist_intermediaryTerms hist_instruc hist_usedfullvars hist_plane_properties hist_turbulence hist_icing];
        
        %saving_name = [simulationDirPath '/' char(flightNames(i)) '_simulated.csv'];
        saving_name = fullfile(simulationDirPath, [char(flightNames(i)) '_simulated.csv']);
        T = array2table(to_save, 'VariableNames', var_names);
        writetable(T, saving_name);

        disp(['Ok for i = ' num2str(i)])
        disp(['Fligth : ' flightNames(i)])
        disp(' ')
    else
        sprintf('Flight %s (n°%i) already simulated.', flightNames(i), i)
    end
end

% Closing the parallel pool
if runParallel
    delete(gcp('nocreate'));
end
