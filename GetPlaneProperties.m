%   Copyright 2024 by Charles Dampeyrou

function [Ixx, Iyy, Izz, Ixz, S, b, cBar, sfc] = GetPlaneProperties(m, Mach)
% This function returns the geometry, mass and specific fuel
%consumption of the aircraft given the mass of the aircraft and the Mach
% Inputs :
%   m - mass of the aircraft, kg
%   Mach - Mach number
% Outputs :
%   Ixx - Roll Moment of Inertia, kg-m^2
%   Iyy - Pitch Moment of Inertia, kg-m^2
%   Izz - Yaw Moment of Inertia, kg-m^2
%   Ixz - Nose-high(low) Product of Inertia, kg-m^2
%   S - Reference Area, m^2
%   b - Wing Span, m
%   cBar - Mean Aerodynamic Chord, m
%   sfc - Specific fuel consumption of the engines, kg/(N.s)

    %	Typical Mass and Inertial Properties
	Ixx		=	35926.5*m/4536;			% Roll Moment of Inertia, kg-m^2
	Iyy		=	33940.7*m/4536;			% Pitch Moment of Inertia, kg-m^2
	Izz		=	67085.5*m/4536;			% Yaw Moment of Inertia, kg-m^2
	Ixz		=	3418.17*m/4536;			% Nose-high(low) Product of Inertia, kg-m^2
	
%	Geometric Properties
	cBar	=	2.14;				% Mean Aerodynamic Chord, m
	b		=	10.4;				% Wing Span, m
	S		=	21.5;				% Reference Area, m^2
	ARw		=	5.02;				% Wing Aspect Ratio
	taperw	=	0.507;				% Wing Taper Ratio
	sweepw	=	13 * .01745329;		% Wing 1/4-chord sweep angle, rad
	ARh		=	4;					% Horizontal Tail Aspect Ratio
	sweeph	=	25 * .01745329;		% Horiz Tail 1/4-chord sweep angle, rad
	ARv		=	0.64;				% Vertical Tail Aspect Ratio
	sweepv	=	40 * .01745329;		% Vert Tail 1/4-chord sweep angle, rad
	lvt		=	4.72;				% Vert Tail Length, m
    
%   Specific fuel consumption of the engines (ref :
%   https://link.springer.com/content/pdf/bbm:978-1-4614-3532-7/1.pdf for the sfc of the engine CJ610-4)
%   a linear variation of the sfc with the Mach number seems to be a good first approximation ref :
%   https://depozit.isae.fr/theses/2005/2005_Roux_Elodie.pdf
    sfcMax = 28.04e-6;             % ref engine SFC in kg/(N.s)
    sfc = sfcMax * (0.8 + 0.2*Mach);