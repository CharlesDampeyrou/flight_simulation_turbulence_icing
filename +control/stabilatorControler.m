classdef stabilatorControler < handle
    % Class to control the stabilator of the airplane. The diagram of the controler can be found in the control diagram files
    
    properties
        PID
        RangeLimiter
    end
    
    methods
        function obj = stabilatorControler(params)
            % params = [
            % pid_ki : integral gain of the PID,
            % pid_delta_t : period at which the controlers are called,
            % pid_initial_output_value : initial value of the stabilator command, in rad,
            % stabilator_min : minimal stabilator command, in rad,
            % stabilator_max : maximal stabilator command, in rad,
            %]
            % The controler uses only the integral part, no proportionnal
            % or derivative part
            import control.pidControler
            import control.rangeLimiter
            pid_kp = 0;
            pid_ki = params(1);
            pid_kd = 0;
            pid_delta_t = params(2);
            pid_initial_output_value = params(3);
            pid_initial_input_value = 0;
            stabilator_min = params(4);
            stabilator_max = params(5);
            obj.PID = pidControler(...
                pid_kp, pid_ki, pid_kd, pid_delta_t, ...
                pid_initial_output_value, pid_initial_input_value, ...
                stabilator_min, stabilator_max);
            obj.RangeLimiter = rangeLimiter(...
                stabilator_min, stabilator_max);
        end
        
        function u_stabilator = step(obj, u_elevator)
            % This method gives the stabilisator command using the elevator
            % command
            % Parameters :
            %   - u_elevator : elevator command in rad
            % Returns :
            %   - u_stabilator, stabilator command in rad
            % The control diagram can be found in the control diagram files
            elevator_derivative = 0;
            pid_output = obj.PID.step(u_elevator, elevator_derivative);
            u_stabilator = obj.RangeLimiter.step(pid_output);
        end
    end
end

