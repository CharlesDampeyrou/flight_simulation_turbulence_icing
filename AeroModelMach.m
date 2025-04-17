	function [CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroModelMach(x,u,Mach,alphar,betar,V)
%	Aerodynamic Coefficients of the Aircraft, Thrust Model,
%	and Geometric and Inertial Properties

%   BizJet A
%   ========
%	June 12, 2015  
%	===============================================================
%	Copyright 2006-2015 by ROBERT F. STENGEL.  All rights reserved.
%   Copyright 2024 by Charles Dampeyrou

%	Business Jet -- Low-Angle-of-Attack, Mach-Dependent Model
%	Landing Gear: Up or Down (GEAR  = 1 or 0)
%	Flap Setting, x(14): between 0 deg and 38 deg (0 or (38*pi/180) rad)
%   The function is modified to accept flap settings between 0 and 38deg,
%   the aerodynamic coefficients are interpolated linearly.
%	Symmetric Spoiler: Deployed or Closed (SPOIL = 1 or 0)
    
    GEAR = x(16);
    SPOIL = 0; % Spoiler is supposed to be closed
	
%	Geometric Properties
    m = x(13);
    [Ixx, Iyy, Izz, Ixz, S, b, cBar, sfc] = GetPlaneProperties(m, Mach);
	cBar	=	2.14;				% Mean Aerodynamic Chord, m
	ARw		=	5.02;				% Wing Aspect Ratio
	taperw	=	0.507;				% Wing Taper Ratio
	sweepw	=	13 * .01745329;		% Wing 1/4-chord sweep angle, rad
	ARh		=	4;					% Horizontal Tail Aspect Ratio
	sweeph	=	25 * .01745329;		% Horiz Tail 1/4-chord sweep angle, rad
	ARv		=	0.64;				% Vertical Tail Aspect Ratio
	sweepv	=	40 * .01745329;		% Vert Tail 1/4-chord sweep angle, rad
	lvt		=	4.72;				% Vert Tail Length, m
	
%	Thrust Properties
	StaticThrust	=	26243.2;	% Static Thrust @ Sea Level, N
	
%	Current Thrust
	[airDens,airPres,temp,soundSpeed] = Atmos(-x(6));
	Thrust			=	u(4) * StaticThrust * (airDens / 1.225)^0.7 ...
						* (1 - exp((-x(6) - 17000) / 2000));
									% Thrust at Altitude, N
	
%	Current Mach Effects, normalized to Test Condition B (Mach = 0.1734)
	PrFac			=	1 / (sqrt(1 - Mach^2) * 1.015);	
									% Prandtl Factor
	WingMach		=	1 / ((1 + sqrt(1 + ((ARw/(2 * cos(sweepw)))^2) ...
						* (1 - Mach^2 * cos(sweepw)))) * 0.268249);
									% Modified Helmbold equation
	HorizTailMach	=	1 / ((1 + sqrt(1 + ((ARh/(2 * cos(sweeph)))^2) ...
						* (1 - Mach^2 * cos(sweeph)))) * 0.294539);
									% Modified Helmbold equation
	VertTailMach	=	1 / ((1 + sqrt(1 + ((ARv/(2 * cos(sweepv)))^2) ...
						* (1 - Mach^2 * cos(sweepv)))) * 0.480338);
									% Modified Helmbold equation
	
%	Current Longitudinal Characteristics
%	====================================
%	Lift Coefficient
	CLo		=	0.1095 - GEAR * 0.0192;				% Zero-AoA Lift Coefficient (B)
	%if GEAR >= 1
	%	CLo	=	CLo - 0.0192;		% Gear-down correction
	%end
	%if u(6) >= 0.65
	%	CLo	=	CLo + 0.5182;		% 38 deg-flap correction
    %end
    CLo = CLo + 0.5182*x(14)/(38*pi/180);
	if SPOIL >= 1
		CLo	=	CLo - 0.1897;		% 42 deg-Symmetric Spoiler correction
	end	
	
	CLar	=	5.6575;				% Lift Slope (B), per rad
	%if u(6) >= 0.65
	%	CLar	=	CLar - 0.0947;
	%end
    CLar	=	CLar - 0.0947*x(14)/(38*pi/180);

	CLqr    =	4.231 * cBar / (2 * V);
									% Pitch-Rate Effect, per rad/s
	
	CLdSr	=	1.08;				% Stabilator Effect, per rad
	%if u(6) >= 0.65
	%	CLdSr	=	CLdSr - 0.4802;	% 38�-flap correction
	%end
    CLdSr	=	CLdSr - 0.4802*x(14)/(38*pi/180);
	
	CLdEr	=	0.5774;				% Elevator Effect, per rad
	%if u(6) >= 0.65
	%	CLdEr	=	CLdEr - 0.2665;	% 38 deg-flap correction
	%end
    CLdEr	=	CLdEr - 0.2665*x(14)/(38*pi/180);

    CLoIceEffect = 1 - 0.06*x(15); % Evolution of the lift due to the icing of the aircraft
    CLarIceEffect = 1-0.08*x(15);
    CLo = CLo * CLoIceEffect;
    CLar = CLar * CLarIceEffect;
	CL	=	CLo + (CLar*alphar + CLqr*x(8) + CLdSr*u(7) + CLdEr*u(1)) ...
			* WingMach;
                                    % Total Lift Coefficient, w/Mach Correction
                                    
	
%	Drag Coefficient
	CDo		=	0.0255 + GEAR * 0.0191;				% Parasite Drag Coefficient (B)
	%if GEAR >= 1
	%	CDo	=	CDo + 0.0191;		% Gear-down correction
	%end
	%if u(6) >= 0.65
	%	CDo	=	CDo + 0.0836;		% 38 deg-flap correction
	%end
    CDo	=	CDo + 0.0836*x(14)/(38*pi/180);
	if SPOIL >= 1
		CDo	=	CDo + 0.0258;		% 42 deg-Symmetric Spoiler correction
	end	

	%epsilon	=	0.0718;				% Induced Drag Factor
	%if u(6) >= 0.65
	%	epsilon	=	0.079;			% 38 deg-flap correction
	%end
    epsilon = 0.0718 + 0.0072*x(14)/(38*pi/180);
    CDoIceEffect = 1 + 0.4*x(15); % Effect of icing on CDo
    epsilonIceEffect = 1 + 0.78*x(15); % Effect of icing on epsilon
    CDo = CDo * CDoIceEffect;
    epsilon = epsilon * epsilonIceEffect;
	CD	=	CDo * PrFac + epsilon * CL^2;
                                    % Total Drag Coefficient, w/Mach Correction
	
%	Pitching Moment Coefficient
	Cmo		=	0 + GEAR * 0.0255;					% Zero-AoA Moment Coefficient (B)
	%if GEAR >= 1
	%	Cmo	=	Cmo + 0.0255;		% Gear-down correction
	%end
	%if u(6) >= 0.65
	%	Cmo	=	Cmo - 0.058;		% 38 deg-flap correction
	%end
    Cmo	=	Cmo - 0.058*x(14)/(38*pi/180);
	if SPOIL >= 1
		Cmo	=	Cmo - 0.0154;		% 42 deg-Symmetric Spoiler correction
	end	
	
	Cmar	=	-1.231;				% Static Stability (B), per rad
	%if u(6) >= 0.65
	%	Cmar	=	Cmar + 0.0138;
	%end
    Cmar	=	Cmar + 0.0138*x(14)/(38*pi/180);

	Cmqr    =	 -18.8 * cBar / (2 * V);
                                    % Pitch-Rate + Alpha-Rate Effect, per rad/s
	
	CmdSr	=	-2.291;				% Stabilator Effect, per rad
	%if u(6) >= 0.65
	%	CmdSr	=	CmdSr + 0.121;	% 38 deg-flap correction
	%end
    CmdSr	=	CmdSr + 0.121*x(14)/(38*pi/180);
	
	CmdEr	=	-1.398;				% Elevator Effect, per rad
	%if u(6) >= 0.65
	%	CmdEr	=	CmdEr + 0.149;	% 38 deg-flap correction
	%end
    CmdEr	=	CmdEr + 0.149*x(14)/(38*pi/180);

	Cm	=	Cmo + (Cmar*alphar + Cmqr*x(8) + CmdSr*u(7) + CmdEr*u(1)) ...
			* HorizTailMach;
                                    % Total Pitching Moment Coefficient, w/Mach Correction
	
%	Current Lateral-Directional Characteristics
%	===========================================

%	Side-Force Coefficient
	CYBr	=	-0.7162;			% Side-Force Slope (B), per rad
	%if u(6) >= 0.65
	%	CYBr	=	CYBr + 0.0826;
	%end
    CYBr	=	CYBr + 0.0826*x(14)/(38*pi/180);

	CYdAr	=	-0.00699;			% Aileron Effect, per rad
	
	CYdRr	=	0.1574;				% Rudder Effect, per rad
	%if u(6) >= 0.65
	%	CYdRr	=	CYdRr - 0.0093;	% 38 deg-flap correction
	%end
    CYdRr	=	CYdRr - 0.0093*x(14)/(38*pi/180);
	
	CYdASr	=	0.0264;				% Asymmetric Spoiler Effect, per rad
	%if u(6) >= 0.65
	%	CYdASr	=	CYdASr + 0.0766;	
									% 38 deg-flap correction
	%end
    CYdASr	=	CYdASr + 0.0766*x(14)/(38*pi/180);

	CY	=	(CYBr*betar + CYdRr*u(3)) * VertTailMach ... 
			+ (CYdAr*u(2) + CYdASr*u(5)) * WingMach;
                                    % Total Side-Force Coefficient, w/Mach Correction

%	Yawing Moment Coefficient
	CnBr	=	0.1194;				% Directional Stability (B), per rad
	%if u(6) >= 0.65
	%	CnBr	=	CnBr - 0.0092;
	%end
    CnBr	=	CnBr - 0.0092*x(14)/(38*pi/180);

	Cnpr	=	CL * (1 + 3 * taperw)/(12 * (1 + taperw)) * (b / (2 * V));				
									% Roll-Rate Effect, per rad/s
	
	Cnrr	=	(-2 * (lvt / b) * CnBr * VertTailMach - 0.1 * CL^2) ...
				* (b / (2 * V));				
									% Yaw-Rate Effect, per rad/s

	CndAr	=	0;                      % Aileron Effect, per rad
	%if u(6) >= 0.65
	%	CndAr	=	CndAr + 0.0028;
	%end
    CndAr	=	CndAr + 0.0028*x(14)/(38*pi/180);
	
	CndRr	=	-0.0713;                % Rudder Effect, per rad
	%if u(6) >= 0.65
	%	CndRr	=	CndRr - 0.0185;     % 38 deg-flap correction
	%end
    CndRr	=	CndRr - 0.0185*x(14)/(38*pi/180);
	
	CndASr	=	-0.0088;                % Asymmetric Spoiler Effect, per rad
	%if u(6) >= 0.65
	%	CndASr	=	CndASr - 0.0106;	
                                        % 38 deg-flap correction
	%end
    CndASr	=	CndASr - 0.0106*x(14)/(38*pi/180);

	Cn	=	(CnBr*betar + CndRr*u(3)) * VertTailMach ...
			+ Cnrr * x(9) + Cnpr * x(7) ...
			+ (CndAr*u(2) + CndASr*u(5)) * WingMach;
                                        % Total Yawing-Moment Coefficient, w/Mach Correction

%	Rolling Moment Coefficient
	ClBr	=	-0.0918;                % Dihedral Effect (B), per rad
	%if u(6) >= 0.65
	%	ClBr	=	ClBr - 0.0092;
	%end
    ClBr	=	ClBr - 0.0092*x(14)/(38*pi/180);

	Clpr	=	-CLar * (1 + 3 * taperw)/(12 * (1 + taperw)) ...
				* (b / (2 * V));				
                                        % Roll-Rate Effect, per rad/s
	
	Clrr	=	(CL * (1 + 3 * taperw)/(12 * (1 + taperw)) ...
				* ((Mach * cos(sweepw))^2 - 2) / ((Mach * cos(sweepw))^2 - 1)) ...
				* (b / (2 * V));				
                                        % Yaw-Rate Effect, per rad/s

	CldAr	=	0.1537;                 % Aileron Effect, per rad
	%if u(6) >= 0.65
	%	CldAr	=	CldAr + 0.01178;
	%end
    CldAr	=	CldAr + 0.01178*x(14)/(38*pi/180);
	
	CldRr	=	0.01208;				% Rudder Effect, per rad
	%if u(6) >= 0.65
	%	CldRr	=	CldRr + 0.01115;	% 38 deg-flap correction
	%end
    CldRr	=	CldRr + 0.01115*x(14)/(38*pi/180);
	
	CldASr	=	-0.01496;				% Asymmetric Spoiler Effect, per rad
	%if u(6) >= 0.65
	%	CldASr	=	CldASr - 0.02376;	
                                        % 38 deg-flap correction
	%end
    CldASr	=	CldASr - 0.02376*x(14)/(38*pi/180);

	Cl	=	(ClBr*betar + CldRr*u(3)) * VertTailMach ... 
			+ Clrr * x(9) + Clpr * x(7) ...
			+ (CldAr*u(2) + CldASr*u(5)) * WingMach;
                                        % Total Rolling-Moment Coefficient, w/Mach Correction

