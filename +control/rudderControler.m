classdef rudderControler < handle
    % Class to control the drift angle of the airplane. The diagram of the controler can be found in the control diagram files
    
    properties
        PID
        RangeLimiter
    end
    
    methods
        function obj = rudderControler(params)            
            %params = [
            % pid_kp : proportional gain of the PID,
            % pid_ki,
            % pid_kd,
            % pid_delta_t : period at which the controlers are called,
            % rudder_max_value : maximal rudder angle, in rad,
            %]
            import control.pidControler
            import control.rangeLimiter
            pid_kp = params(1);
            pid_ki = params(2);
            pid_kd = params(3);
            pid_delta_t = params(4);
            rudder_max_value = params(5);
            max_airspeed=240;
            obj.PID = pidControler(pid_kp, pid_ki, pid_kd, ...
                pid_delta_t, 0, 0, -rudder_max_value*max_airspeed^2, ...
                rudder_max_value*max_airspeed^2);
            obj.RangeLimiter = rangeLimiter(-rudder_max_value, ...
                                                  rudder_max_value);
        end
        
        function u_rudder = step(obj, betar, rr, airspeed)
            % Method to obtain the rudder instruction using the drift and
            % and yaw rate.
            % Parameters :
            %   - betar : drift angle in rad
            %   - rr : yaw rate in rad/s
            %   - airspeed in m/s
            % Returns :
            %   - u_rudder, rudder command in rad
            % The control diagram can be found in the control diagram files
            pid_output = obj.PID.step(-betar, rr);
            u_rudder = pid_output / (airspeed^2);
            u_rudder = obj.RangeLimiter.step(u_rudder);
        end
    end
end

