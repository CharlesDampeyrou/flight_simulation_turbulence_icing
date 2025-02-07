function J = TrimCostIcing(x, u, V, OptParam)
%	FLIGHT Cost Function for Longitudinal Trim in Steady Level Flight
%	June 12, 2015   
%	===============================================================
%	Copyright 2006-2015 by ROBERT F. STENGEL.  All rights reserved.
%   Updated in 2024 by Charles Dampeyrou in order to incorporate icing and intermediate position of the gear and flaps

	R	=	[1 0 0
			0 1 0
			0 0 1];

% Optimization Vector:
%	1 = Stabilator, rad
%	2 = Throttle, %
%	3 = Pitch Angle, rad
				
	u	=	[u(1)
			u(2)
			u(3)
			OptParam(2)
			u(5)
			u(6)
			OptParam(1)
            u(7)];
				
	x	=	[V * cos(OptParam(3))
			x(2)
			V * sin(OptParam(3))
			x(4)
			x(5)
			x(6)
			x(7)
			x(8)
			x(9)
			x(10)
			OptParam(3)
			x(12)
            x(13)
            x(14)
            x(15)
            x(16)];
	
	% creation of a turbulence data matrix without any perturbation and an icing matrix without any icing condition
    turbData = zeros(2,8);
    turbData(2,1) = 2; % maximal time = 2s
    icingData = zeros(2,2);
    icingData(2,1) = 2; % maximal time = 2s
	xdot	=	EoM(1,x,u, turbData, icingData); % EoM called at t=1s
	xCost	=	[xdot(1)
				xdot(3)
				xdot(8)];
	J		=	xCost' * R * xCost;