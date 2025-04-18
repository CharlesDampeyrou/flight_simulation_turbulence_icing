	function xdot = EoM(t,x, u, turbData, icingData)
%	FLIGHT Equations of Motion

%	June 12, 2015  
%	===============================================================
%	Copyright 2006-2015 by ROBERT F. STENGEL.  All rights reserved.
%   Copyright 2024 by Charles Dampeyrou

%   The code from Robert F. Stengel was modified in order to incorporate icing.

% inputs:
%   t       - time
%   x : state vector
%       x(1)    - u velocity component in body axis
%       x(2)    - Body-axis y inertial velocity, vb, m/s
%		x(3)    - Body-axis z inertial velocity, wb, m/s
%		x(4)    - North position of center of mass WRT Earth, xe, m
%		x(5)    - East position of center of mass WRT Earth, ye, m
%		x(6)    - Negative of c.m. altitude WRT Earth, ze = -h, m
%		x(7)    - Body-axis roll rate, pr, rad/s
%		x(8)    - Body-axis pitch rate, qr, rad/s
%		x(9)    - Body-axis yaw rate, rr,rad/s
%		x(10)   - Roll angle of body WRT Earth, phir, rad
%		x(11)   - Pitch angle of body WRT Earth, thetar, rad
%		x(12)   - Yaw angle of body WRT Earth, psir, rad
%       x(13)   - Mass of the aircraft, m, kg
%       x(14)   - Position of the flaps, fl, rad
%   u : control vector
%       u(1)    - Elevator, dEr, rad, positive: trailing edge down
%       u(2)    - Aileron, dAr, rad, positive: left trailing edge down
%		u(3)    - Rudder, dRr, rad, positive: trailing edge left
%		u(4)    - Throttle, dT, %
%		u(5)    - Asymmetric Spoiler, dASr, rad
%       u(6)    - Flap command, dFr, rad
%		u(7)    - Stabilator, dSr, rad
%       u(8)    - Gear command, boolean, 1 for extended
%   turbData - turbulence data readable by getTurbulence
%   icingData - icing data, first column is time, second is the icing level
    
    m = x(13);
    icing = interp1(icingData(:,1), icingData(:,2), t, 'previous');

    AeroModel = @AeromodelMach;
    
%	Earth-to-Body-Axis Transformation Matrix
	HEB		=	DCM(x(10),x(11),x(12));
%	Atmospheric State
    x(6)    =   min(x(6),0);    % Limit x(6) to <= 0 m
	[airDens,airPres,temp,soundSpeed]	=	Atmos(-x(6));
%	Body-Axis Wind Field
	windb	=	WindField(-x(6),x(10),x(11),x(12));
    turb = getTurbulence(t,turbData);
%	Body-Axis Gravity Components
	gb		=	HEB * [0;0;9.80665];

%	Air-Relative Velocity Vector
    x(1)    =   max(x(1),0);        %   Limit axial velocity to >= 0 m/s
	Va		=	[x(1);x(2);x(3)] + windb + turb(2:4);
	V		=	sqrt(Va' * Va);
	alphar	=	atan(Va(3) / abs(Va(1)));
 %   alphar  =   min(alphar, (pi/2 - 1e-6));  %   Limit angle of attack to <= 90 deg
    
	alpha 	=	57.2957795 * alphar;
    
	betar	= 	asin(Va(2) / V);
	beta	= 	57.2957795 * betar;
	Mach	= 	V / soundSpeed;
	qbar	=	0.5 * airDens * V^2;
    
%   
    [Ixx, Iyy, Izz, Ixz, S, b, cBar, sfc] = GetPlaneProperties(m, Mach);
    
%   Addition of the effect of the turbulence angular rates
    xTurb = x;
    xTurb(7:9) = x(7:9) + turb(5:7);
    
%	Force and Moment Coefficients; Thrust	
	[CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroModelMach(xTurb,u,Mach,alphar,betar,V);

	qbarS	=	qbar * S;

	CX	=	-CD * cos(alphar) + CL * sin(alphar);	% Body-axis X coefficient
	CZ	= 	-CD * sin(alphar) - CL * cos(alphar);	% Body-axis Z coefficient

%	State Accelerations
	Xb =	(CX * qbarS + Thrust) / m;
	Yb =	CY * qbarS / m;
	Zb =	CZ * qbarS / m;
	Lb =	Cl * qbarS * b;
	Mb =	Cm * qbarS * cBar;
	Nb =	Cn * qbarS * b;
	nz	=	-Zb / 9.80665;							% Normal load factor

%	Dynamic Equations
	xd1 = Xb + gb(1) + x(9) * x(2) - x(8) * x(3);
	xd2 = Yb + gb(2) - x(9) * x(1) + x(7) * x(3);
	xd3 = Zb + gb(3) + x(8) * x(1) - x(7) * x(2);
	
	y	=	HEB' * [x(1);x(2);x(3)];
	xd4	=	y(1);
	xd5	=	y(2);
	xd6	=	y(3);
	
	xd7	= 	(Izz * Lb + Ixz * Nb - (Ixz * (Iyy - Ixx - Izz) * x(7) + ...
				(Ixz^2 + Izz * (Izz - Iyy)) * x(9)) * x(8)) / (Ixx * Izz - Ixz^2);
	xd8 = 	(Mb - (Ixx - Izz) * x(7) * x(9) - Ixz * (x(7)^2 - x(9)^2)) / Iyy;
	xd9 =	(Ixz * Lb + Ixx * Nb + (Ixz * (Iyy - Ixx - Izz) * x(9) + ...
				(Ixz^2 + Ixx * (Ixx - Iyy)) * x(7)) * x(8)) / (Ixx * Izz - Ixz^2);

	cosPitch	=	cos(x(11));
	if abs(cosPitch)	<=	0.00001
		cosPitch	=	0.00001 * sign(cosPitch);
	end
	tanPitch	=	sin(x(11)) / cosPitch;
		
	xd10	=	x(7) + (sin(x(10)) * x(8) + cos(x(10)) * x(9)) * tanPitch;
	xd11	=	cos(x(10)) * x(8) - sin(x(10)) * x(9);
	xd12	=	(sin(x(10)) * x(8) + cos(x(10)) * x(9)) / cosPitch;
    
    xd13    =   - Thrust * sfc; % weigth loss due to fuel consumption
    
    % Evolution of the position of the flaps due to the flap command
    if u(6)>=0.66 && x(14)<u(6)
        xd14 = 38*pi/180 / 10; % flap output speed to extend the flaps in 10s
    elseif u(6)<=1*pi/180 && x(14)>u(6)
        xd14 = - 38*pi/180 / 10;
    else
        xd14 = 0;
    end
    
    % icing and deicing speed
    if icing~=0 && x(15)<icing
        xd15 = 1 / 60; % The icing occures in 1min.
    elseif icing==0 && x(15)>0
        xd15 = - 1 / 60; % The deicing occurs in 1min.
    else
        xd15 = 0;
    end
    
    % Evolution of the position of the GEAR
    if u(8) ~= 0 && x(16)<1
        xd16 = 1 / 5; % The gear extends in 5s
    elseif u(8) == 0 && x(16) > 0
        xd16 = -1 / 5;
    else 
        xd16 = 0;
    end
	
	xdot	=	[xd1;xd2;xd3;xd4;xd5;xd6;xd7;xd8;xd9;xd10;xd11;xd12;xd13;xd14;xd15; xd16];
