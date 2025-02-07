function [x_hist, u_hist, intermediary_terms_hist, hist_usedfullvars] = ...
    compute_test_trajectory(initial_state, initial_command, controler, ...
    instruction_hist, turb_hist, delta_t)
% This function return the historic of the airplanes state and commands
% using the controler provided and an historic of instructions
% Parameters :
%   - initial_state : the state x in which the simulation begins
%   - initial_command : the command u in which the simulation begins
%   - controler : the controler used to compute the commands
%   - instruction_hist : the historic of instructions
%   - turb_hist : the historic of turbulences perturbations
%   - delta_t : the time step of the simulation
% Returns :
%   - x_hist : the historic of the states
%   - u_hist : the historic of the commands
%   - intermediary_terms_hist : the historic of the intermediary terms of the controler
%   - hist_usedfullvars : the historic of the useful variables of the simulation
t_end = instruction_hist(end, 1);
nb_steps = t_end/delta_t;
x_hist = zeros(nb_steps+1, 16);
x_hist(1,:) = initial_state;
u_hist = zeros(nb_steps, 8);
intermediary_terms_hist = zeros(nb_steps, 2);
hist_usedfullvars = zeros(nb_steps, 24);
x = initial_state;
u = initial_command;
icing_hist = [0,0;t_end,0];
MODEL = 1;

for i=1:nb_steps
    t = i*delta_t;
    instr = interp1(instruction_hist(:, 1), instruction_hist(:, 2:end), t);
    turb = getTurbulence(t, turb_hist);
    [CD,CL,CY,Cl,Cm,Cn,Thrust,airDens,airPres,temp,V,Mach,alphar,betar, ...
        gammar,nx,ny,nz,xdot1,xdot2,xdot3,xdot7,xdot8,xdot9] = ...
        GetUsefullVars(x, u, t, turb_hist, icing_hist, MODEL);
    [u, intermediary_terms] = controler.step(x_hist(i, :), turb, instr(1), instr(2), instr(3), instr(4));
    [~, xinteg] = ode15s(@(t, x) EoM(t,x,u, turb_hist, icing_hist), ...
        [t, t+delta_t], x);
    x = xinteg(end, :)';
    x_hist(i+1, :) = x';
    u_hist(i, :) = u';
    intermediary_terms_hist(i, :) = intermediary_terms';
    hist_usedfullvars(i,:) = [CD,CL,CY,Cl,Cm,Cn,Thrust,airDens,airPres,...
        temp,V,Mach,alphar,betar,gammar,nx,ny,nz,xdot1,xdot2,xdot3,...
        xdot7,xdot8,xdot9];
end

