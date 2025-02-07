%   Copyright 2024 by Charles Dampeyrou


function [x_init, u_init] = get_initial_state(initial_tas, ...
    initial_altitude, initial_heading, initial_mass, initial_flaps, ...
    initial_icing, initial_gear)
% This function returns the initial state and command of the airplane to
% have a stabilized fligth at the wanted conditions
% inputs:
%   initial_tas - initial true airspeed, m/s
%   initial_altitude - initial altitude, m
%   initial_heading - initial heading, rad
%   initial_mass - initial mass, kg
%   initial_flaps - initial flaps position, rad
%   initial_icing - initial icing accretion on the airplane, 0 to 1
%   initial_gear - initial gear position, 0 for retracted, 1 for extended
% outputs:
%   x_init - initial state vector
%   u_init - initial control vector

x_init = [initial_tas
    0 
    0 % Vitesse initiale uniquement selon l'axe avion
    0
    0
    - initial_altitude
    0
    0
    0
    0
    0
    initial_heading
    initial_mass
    initial_flaps
    initial_icing
    initial_gear];
u_init = [0
    0
    0
    0.5
    0
    initial_flaps
    0
    initial_gear];
opt_param = [0, 0.5, 0]; % stabilator, throttle and pitch angle
options = optimset('MaxIter',1000,'MaxFunEvals',1000,'TolFun',1e-14);
[opt_param, ~, ~, ~]  =	fminsearch(...
    @(param) TrimCostIcing(x_init, u_init, initial_tas, param),...
    opt_param, ...
    options);
x_init(11) = opt_param(3);
x_init(1) = initial_tas * cos(opt_param(3));
x_init(3) = initial_tas * sin(opt_param(3));

u_init(4) = opt_param(2);
u_init(7) = opt_param(1);
end

