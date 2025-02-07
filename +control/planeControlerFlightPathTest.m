classdef planeControlerFlightPathTest < handle
    % Plane controler controling the flight path angle instead of the altitude.
    % This controler is used only for the tuning of the controlers.
    properties
        FlightPathControler;
        RouteControler;
        RudderControler;
        ThrottleControler;
        FlapsControler;
        StabilatorControler;
        GearControler;
        VarLastState;
        DeltaT
    end
    methods
        function obj = planeControlerFlightPathTest(...
                flight_path_controler_params, throttle_controler_params, ...
                flaps_controler_params, stabilator_controler_params, ...
                gear_controler_params, x0, delta_t)
            import control.flightPathControler
            import control.routeControler
            import control.rudderControler
            import control.throttleControler
            import control.flapsControler
            import control.stabilatorControler
            import control.gearControler
            import control.dummyControler
            obj.FlightPathControler = flightPathControler(...
                flight_path_controler_params);
            obj.RouteControler = dummyControler();
            obj.RudderControler = dummyControler();
            obj.ThrottleControler = throttleControler(throttle_controler_params);
            obj.FlapsControler = flapsControler(flaps_controler_params);
            obj.StabilatorControler = stabilatorControler(stabilator_controler_params);
            obj.GearControler = gearControler(gear_controler_params);
            plane_speed_E = DCM(x0(10), x0(11), x0(12)) \ [x0(1), x0(2), x0(3)]';
            gamma = asin(-plane_speed_E(3) / sqrt(...
                plane_speed_E(1)^2+plane_speed_E(2)^2+plane_speed_E(3)^2));
            initial_speed = sqrt(x0(1)^2+x0(2)^2+x0(3)^3);
            obj.VarLastState = [-x0(6), gamma, initial_speed]; % [h, gamma, airspeed] at t-1
            obj.DeltaT = delta_t;
        end
        
        function [u, intermediary_terms] = step(obj, x, turb, route_target, target_airspeed, target_gamma, flaps_instruction)
            % Method to obtain the complete command from the instructions
            % Parameters :
            %   - x : the plane's last state
            %   - turb : the turbulence at the considered time
            %   - route_target in rad
            %   - target_airspeed in m/s. The airplane will try to reach this
            %   airspeed
            %   - target_gamma in rad
            %   - flap_instruction in rad
            
            % airspeed in the reference frame of the airplane
            air_speed_A = [x(1);x(2);x(3)] + turb(2:4);
            tas = sqrt(air_speed_A' * air_speed_A);
            plane_speed_E = DCM(x(10), x(11), x(12)) \ [x(1), x(2), x(3)]';
            gamma = asin(-plane_speed_E(3) / sqrt(...
                plane_speed_E(1)^2+plane_speed_E(2)^2+plane_speed_E(3)^2));
            
            gamma_d = (gamma - obj.VarLastState(2))/obj.DeltaT;
            gamma_error = target_gamma - gamma;
            gamma_error_d = -gamma_d;
            phir = x(10);
            u_elevator = obj.FlightPathControler.step(...
                gamma_error, gamma_error_d, tas);
            
            u_aileron = obj.RouteControler.step();
            
            u_rudder = obj.RudderControler.step();
            
            tas_error = target_airspeed - tas;
            tas_d = (tas - obj.VarLastState(3))/obj.DeltaT;
            u_throttle = obj.ThrottleControler.step(tas_error, tas_d);
            
            u_asym_spoiler = 0;
            
            h = -x(6);
            u_flap = obj.FlapsControler.step(flaps_instruction, tas, h);
            
            u_stabilator = obj.StabilatorControler.step(u_elevator);
            
            u_gear = obj.GearControler.step(h);
            
            obj.VarLastState = [h, gamma, tas];
            
            u = [u_elevator, u_aileron, u_rudder, u_throttle, ...
                u_asym_spoiler, u_flap, u_stabilator, u_gear];
            intermediary_terms = [target_gamma, 0];
            
        end
    end
end