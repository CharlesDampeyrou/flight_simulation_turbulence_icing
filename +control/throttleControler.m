classdef throttleControler < handle
    % Class controling the thrust using the air speed error. The diagram of the controler can be found in the control diagram files
    
    properties
        PID
        RangeLimiter
    end
    
    methods
        function obj = throttleControler(params)
            %params = [
            % pid_kp : proportional gain of the PID,
            % pid_ki,
            % pid_kd,
            % pid_delta_t : period at which the controlers are called,
            % throttle_initial_value : initial value of the throttle command, in [0, 1],
            %]
            import control.pidControler
            import control.rangeLimiter
            pid_kp = params(1);
            pid_ki = params(2);
            pid_kd = params(3);
            pid_delta_t = params(4);
            throttle_initial_value = params(5);
            throttle_min_value = 0;
            throttle_max_value = 1;
            obj.PID = pidControler(pid_kp, pid_ki, pid_kd, ...
                pid_delta_t, throttle_initial_value, 0, ...
                throttle_min_value, throttle_max_value);
            obj.RangeLimiter = rangeLimiter(throttle_min_value, ...
                throttle_max_value);
        end
        
        function u_throttle = step(obj,tas_error, tas_d)
            % Method to obtain the throttle command using the speed error
            % Parameters :
            %   - tas_error : air speed error in m/s
            %   - tas_d : air speed derivative in m/s^2
            % Returns :
            %   - u_throttle, throttle command in [0, 1]
            pid_output = obj.PID.step(tas_error, -tas_d);
            u_throttle = obj.RangeLimiter.step(pid_output);
            
        end
    end
end

