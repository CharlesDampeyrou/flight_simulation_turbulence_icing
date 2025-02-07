%   Copyright 2024 by Charles Dampeyrou

function [xiTarget, vTarget, hTarget, flapInstruc] = GetInstruction(t, instructions)
% Function to extract the instructions at time t.
% inputs :
%   t - time, s
%   instructions - matrix of instructions with the following columns :
%       instructions(:,1) - time, s
%       instructions(:,3) - heading, deg
%       instructions(:,4) - altitude, m
%       instructions(:,5) - speed, m/s
% outputs :
%   xiTarget - target heading, rad
%   vTarget - target speed, m/s
%   hTarget - target altitude, m
%   flapInstruc - target flaps position, rad
% The flap instruction is 0 if the target speed is above 75 m/s, and 38 deg otherwise.

xiTarget = wrapToPi(interp1(instructions(:,1), unwrap(instructions(:,3)*pi/180), t));
vTarget = interp1(instructions(:,1), instructions(:,5), t, 'makima');
hTarget = interp1(instructions(:,1), instructions(:,4), t);
if vTarget > 75 % Les volets sont sortis pour une vitesse cible de moins de 75m.s^-1
    flapInstruc = 0;
else
    flapInstruc = 38*pi/180;
end
