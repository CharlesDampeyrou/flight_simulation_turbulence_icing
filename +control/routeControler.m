classdef routeControler < handle
    % Class to control the route of the plane. The diagram of the controler can be found in the control diagram files
    
    properties
        RouteErrorRangeLimiter
        RollAnglePID
        RollAngleRangeLimiter
        RollControler
    end
    
    methods
        function obj = routeControler(params)
            % params = [
            % max_route_error : maximum route error considerd by the controler, in rad,
            % roll_angle_pid_kp : proportional gain of the roll angle PID,
            % roll_angle_pid_ki,
            % roll_angle_pid_kd,
            % delta_t : period at which the controlers are called,
            % max_roll_angle : maximal target roll angle, in rad,
            % aileron_pid_kp : proportional gain of the aileron PID,
            % aileron_pid_ki,
            % aileron_pid_kd,
            % max_aileron_angle : maximal aileron angle, in rad,
            import control.pidControler
            import control.rangeLimiter
            import control.rollControler
            max_route_error = params(1);
            roll_angle_pid_kp = params(2);
            roll_angle_pid_ki = params(3);
            roll_angle_pid_kd = params(4);
            delta_t = params(5);
            max_roll_angle = params(6);
            aileron_pid_kp = params(7);
            aileron_pid_ki = params(8);
            aileron_pid_kd = params(9);
            max_aileron_angle = params(10);
            
            obj.RouteErrorRangeLimiter = rangeLimiter(...
                -max_route_error, max_route_error);
            obj.RollAnglePID = pidControler(roll_angle_pid_kp, ...
                roll_angle_pid_ki, roll_angle_pid_kd, delta_t, 0, 0, ...
                -max_roll_angle, max_roll_angle);
            obj.RollAngleRangeLimiter = rangeLimiter(...
                -max_roll_angle, max_roll_angle);
            roll_controler_params = [aileron_pid_kp, aileron_pid_ki, aileron_pid_kd, ...
                delta_t, max_aileron_angle];
            obj.RollControler = rollControler(roll_controler_params);
        end
        
        function [u_aileron, target_roll_angle] = step(...
                obj, route_error, rr, phir, pr, airspeed)
            % Method to obtain the aileron command and the target roll angle from the following 
            % parameters :
            %   - route_error in rad, wrapped to pi
            %   - rr, body axis yaw rate in rad/s
            %   - phir, roll angle in rad
            %   - pr, body axis roll rate in rad/s
            %   - airspeed in m/s
            % Returns :
            %   - u_aileron, aileron command in rad
            %   - target_roll_angle, target roll angle in rad
            % The diagram of the controler can be found in the control diagram files
            route_error = obj.RouteErrorRangeLimiter.step(route_error);
            target_roll_angle = obj.RollAnglePID.step(route_error, -rr);
            target_roll_angle = obj.RollAngleRangeLimiter.step(target_roll_angle);
            roll_angle_error = target_roll_angle - phir;
            u_aileron = obj.RollControler.step(...
                roll_angle_error, pr, airspeed);
        end
    end
end

