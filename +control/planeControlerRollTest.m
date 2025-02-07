

classdef planeControlerRollTest < handle
    % Plane controler controling the roll angle instead of the altitude.
    % This controler is used only for the tuning of the controlers.
    properties
        AltitudeControler;
        RollControler;
        RudderControler;
        ThrottleControler;
        FlapsControler;
        StabilatorControler;
        GearControler;
        VarLastState;
        DeltaT
    end
    methods
        function obj = planeControlerRollTest(...
                altitude_controler_params, roll_controler_params, ...
                rudder_controler_params, throttle_controler_params, ...
                flaps_controler_params, stabilator_controler_params, ...
                gear_controler_params, x0, delta_t)
            import control.altitudeControler
            import control.rollControler
            import control.rudderControler
            import control.throttleControler
            import control.flapsControler
            import control.stabilatorControler
            import control.gearControler
            obj.AltitudeControler = altitudeControler(altitude_controler_params);
            obj.RollControler = rollControler(roll_controler_params);
            obj.RudderControler = rudderControler(rudder_controler_params);
            obj.ThrottleControler = throttleControler(throttle_controler_params);
            obj.FlapsControler = flapsControler(flaps_controler_params);
            obj.StabilatorControler = stabilatorControler(stabilator_controler_params);
            obj.GearControler = gearControler(gear_controler_params);
            plane_speed_E = DCM(x0(10), x0(11), x0(12)) \ [x0(1), x0(2), x0(3)]';
            gamma = asin(-plane_speed_E(3) / sqrt(...
                plane_speed_E(1)^2+plane_speed_E(2)^2+plane_speed_E(3)^2));
            initial_speed = sqrt(x0(1)^2+x0(2)^2+x0(3)^2);
            obj.VarLastState = [-x0(6), gamma, initial_speed]; % [h, gamma, airspeed] at t-1
            obj.DeltaT = delta_t;
        end
        
        function [u, intermediary_terms] = step(obj, x, turb, target_roll_angle, target_airspeed, target_altitude, ...
                flaps_instruction)
            % Method to obtain the complete command from the instructions
            % Parameters :
            %   - x : the plane's last state
            %   - turb : the turbulence at the considered time
            %   - route_target in rad
            %   - speed_target in m/s. The airplane will try to reach this
            %   airspeed
            %   - altitude_target in m
            %   - flap_instruction in rad
            
            % airspeed in the reference frame of the airplane
            air_speed_A = [x(1);x(2);x(3)] + turb(2:4);
            tas = sqrt(air_speed_A' * air_speed_A);
            % plane speed WRT Earth in the reference frame of the Earth
            plane_speed_E = DCM(x(10), x(11), x(12)) \ [x(1), x(2), x(3)]';
            
            h = -x(6);
            delta_h = target_altitude -h; % x(6) = -h
            h_d = (h-obj.VarLastState(1))/obj.DeltaT ;
            gamma = asin(-plane_speed_E(3) / sqrt(...
                plane_speed_E(1)^2+plane_speed_E(2)^2+plane_speed_E(3)^2));
            gamma_d = (gamma-obj.VarLastState(2))/obj.DeltaT;
            alpha = atan(air_speed_A(3) / air_speed_A(1));
            phir = x(10);
            [target_gamma, u_elevator] = obj.AltitudeControler.step(delta_h, h_d, gamma, gamma_d, alpha, tas);
            
            rr = x(9);
            roll_angle_error = target_roll_angle - phir;
            pr = x(7);
            u_aileron = obj.RollControler.step(...
                roll_angle_error, pr, tas);
            
            betar = asin(air_speed_A(2) / tas);
            u_rudder = obj.RudderControler.step(betar, rr, tas);
            
            tas_error = target_airspeed - tas;
            tas_d = (tas - obj.VarLastState(3))/obj.DeltaT;
            u_throttle = obj.ThrottleControler.step(tas_error, tas_d);
            
            u_asym_spoiler = 0;
            
            u_flap = obj.FlapsControler.step(flaps_instruction, tas, h);
            
            u_stabilator = obj.StabilatorControler.step(u_elevator);
            
            u_gear = obj.GearControler.step(h);
            
            obj.VarLastState = [h, gamma, tas];
            
            u = [u_elevator, u_aileron, u_rudder, u_throttle, ...
                u_asym_spoiler, u_flap, u_stabilator, u_gear];
            intermediary_terms = [target_gamma, target_roll_angle];
            
        end
    end
end