function [CD,CL,CY,Cl,Cm,Cn,Thrust,airDens,airPres,temp,V,Mach,alphar,betar,gammar,xir,nx,ny,nz, xdot1, xdot2, xdot3, xdot7, xdot8, xdot9] = GetUsefullVars(x, u, t, turbData, icingData)
% Function to get variables that are useful for the simulation or for further analysis
% Inputs:
%   x: state vector
%   u: control vector
%   t: time
%   turbData: turbulence data
%   icingData: icing data
% Outputs:
%   CD: drag coefficient
%   CL: lift coefficient
%   CY: side force coefficient
%   Cl: rolling moment coefficient
%   Cm: pitching moment coefficient
%   Cn: yawing moment coefficient
%   Thrust: thrust, N
%   airDens: air density, kg/m^3
%   airPres: air pressure, Pa
%   temp: temperature, K
%   V: airspeed, m/s
%   Mach: Mach number
%   alphar: angle of attack, rad
%   betar: sideslip angle, rad
%   gammar: flight path angle, rad
%   xir: route angle, rad
%   nx: load factor in plane's frame x direction
%   ny: load factor in plane's frame y direction
%   nz: load factor in plane's frame z direction
%   xdot1: time derivative of x(1)
%   xdot2: time derivative of x(2)
%   xdot3: time derivative of x(3)
%   xdot7: time derivative of x(7)
%   xdot8: time derivative of x(8)
%   xdot9: time derivative of x(9)
    AeroModel = @AeroModelMach;
    m = x(13); % mass, kg
    S = 21.5; % Reference Area, m^2
%	Atmospheric State
    x(6)    =   min(x(6),0);    % Limit x(6) to <= 0 m
	[airDens,airPres,temp,soundSpeed]	=	Atmos(-x(6));
%	Body-Axis Wind Field
	windb	=	WindField(-x(6),x(10),x(11),x(12));
    turb = getTurbulence(t,turbData);

%	Air-Relative Velocity Vector
    x(1)    =   max(x(1),0);        %   Limit axial velocity to >= 0 m/s
	Va		=	[x(1);x(2);x(3)] + windb + turb(2:4);
	V		=	sqrt(Va' * Va);
	alphar	=	atan(Va(3) / abs(Va(1)));
 %   alphar  =   min(alphar, (pi/2 - 1e-6));  %   Limit angle of attack to <= 90 deg
    
	betar	= 	asin(Va(2) / V);
    plane_speed_E = DCM(x(10), x(11), x(12)) \ [x(1), x(2), x(3)]';
    gammar = asin(-plane_speed_E(3) / sqrt(...
        plane_speed_E(1)^2+plane_speed_E(2)^2+plane_speed_E(3)^2));
    xir = atan2(plane_speed_E(2), plane_speed_E(1));
	Mach	= 	V / soundSpeed;
    qbar	=	0.5 * airDens * V^2;
    
%	Force and Moment Coefficients; Thrust
    xTurb = x;
    xTurb(7:9) = x(7:9) + turb(5:7);
	[CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroModel(xTurb,u,Mach,alphar,betar,V);
    
    qbarS	=	qbar * S;

	CX	=	-CD * cos(alphar) + CL * sin(alphar);	% Body-axis X coefficient
	CZ	= 	-CD * sin(alphar) - CL * cos(alphar);	% Body-axis Z coefficient

%	Load factors
	Xb =	(CX * qbarS + Thrust) / m;
	Yb =	CY * qbarS / m;
	Zb =	CZ * qbarS / m;
    nx = Xb / 9.80665;
    ny = Yb / 9.80665;
	nz = -Zb / 9.80665;
    
%   State acceleration
    xdot = EoM(t,x, u, turbData, icingData);
    xdot1 = xdot(1);
    xdot2 = xdot(2);
    xdot3 = xdot(3);
    xdot7 = xdot(7);
    xdot8 = xdot(8);
    xdot9 = xdot(9);
end

