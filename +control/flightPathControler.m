classdef flightPathControler < handle
    % Class to control the flight path angle of the airplane. The diagram of the controler can be found in the control diagram files
    
    properties
        elevatorPID;
        elevatorRangeLimiter
    end
    
    methods
        function obj = flightPathControler(params)
            % params = [
            % elevator_pid_kp : proportional gain of the elevator PID,
            % elevator_pid_ki,
            % elevator_pid_kd,
            % delta_t : period at which the controlers are called,
            % initial_elevator : initial elevator command,
            % initial_gamma : initial flight path angle,
            % min_elevator : minimal value of the elevator command,
            % max_elevator : maximal value of the elevator command,
            %]
            import control.pidControler
            import control.rangeLimiter
            elevator_pid_kp = params(1);
            elevator_pid_ki = params(2);
            elevator_pid_kd = params(3);
            delta_t = params(4);
            initial_elevator = params(5);
            initial_gamma = params(6);
            min_elevator = params(7);
            max_elevator = params(8);
            max_airspeed = 230;
            obj.elevatorPID = pidControler(elevator_pid_kp, elevator_pid_ki, elevator_pid_kd, delta_t, initial_elevator, initial_gamma, min_elevator*max_airspeed, max_elevator*max_airspeed);
            obj.elevatorRangeLimiter = rangeLimiter(min_elevator, max_elevator);
            
        end
        
        function u_elevator = step(obj, gamma_error, gamma_error_d, airspeed)
            %METHOD to obtain the elevator command using the following parameters :
            %   - gamma_error : error between the desired flight path angle and the current flight path angle
            %   - gamma_error_d : derivative of the error between the desired flight path angle and the current flight path angle
            %   - airspeed : true airspeed in m/s
            % Returns the elevator command in rad
            % The description of the controler can be found in the control diagram files
            elevator_pid_output = -obj.elevatorPID.step(gamma_error, gamma_error_d);
            u_elevator = elevator_pid_output/(airspeed^2);
            u_elevator = obj.elevatorRangeLimiter.step(u_elevator);
        end
    end
end

