classdef rollControler < handle
    % Class to control the roll angle of the airplane using the ailerons.
    properties
        AileronPID
        AileronRangeLimiter
    end
    
    methods
        function obj = rollControler(params)
            % params = [
            % aileron_pid_kp : proportional gain of the aileron PID,
            % aileron_pid_ki,
            % aileron_pid_kd,
            % delta_t : period at which the controlers are called,
            % max_aileron_angle : maximal value of the aileron command in rad,
            %]
            import control.pidControler
            import control.rangeLimiter
            aileron_pid_kp = params(1);
            aileron_pid_ki = params(2);
            aileron_pid_kd = params(3);
            delta_t = params(4);
            max_aileron_angle = params(5);
            max_airspeed = 240;
            obj.AileronPID = pidControler(aileron_pid_kp, ...
                aileron_pid_ki, aileron_pid_kd, delta_t, 0, 0, ...
                -max_aileron_angle*max_airspeed^2, ...
                max_aileron_angle*max_airspeed^2);
            obj.AileronRangeLimiter = rangeLimiter(-max_aileron_angle, ...
                max_aileron_angle);
        end
        
        function u_aileron = step(obj,roll_angle_error, pr, airspeed)
            % Method to obtain the aileron command from the following 
            % parameters :
            %   - roll_angle_error in rad
            %   - pr, body axis roll rate in rad/s
            %   - airspeed, true airspeed in m/s
            % Returns the aileron command in rad
            pid_output = obj.AileronPID.step(roll_angle_error, -pr);
            u_aileron = pid_output / (airspeed^2);
            u_aileron = obj.AileronRangeLimiter.step(u_aileron);
        end
    end
end

