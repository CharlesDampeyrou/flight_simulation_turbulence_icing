classdef altitudeControler < handle
    % Class to control the altitude of the plane. The diagram of the controler can be found in the control diagram files
    
    properties
        AltitudeErrorRangeLimiter
        GammaPID
        GammaRangeLimiter
        AoaRangeLimiter
        GammaControler
        LastTargetGamma
    end
    
    methods
        function obj = altitudeControler(params)
            % params = [
            % max_altitude_error : max altitude error used as input to the gamma PID,
            % gamma_pid_kp,
            % gamma_pid_ki,
            % gamma_pid_kd,
            % delta_t : period at which the controlers are called,
            % initial_gamma : initial flight path angle,
            % min_gamma : minimal target fligth path angle,
            % max_gamma : maximal target fligth path angle,
            % max_aoa_target : maximal value of the angle of attack to be reached, limits the target gamma,
            % elevator_pid_kp,
            % elevator_pid_ki,
            % elevator_pid_kd,
            % elevator_min : minimal value of the elevator command,
            % elevator_max : maximal value of the elevator command,
            %]
            import control.pidControler
            import control.rangeLimiter
            import control.flightPathControler
            max_altitude_error = params(1);
            gamma_pid_kp = params(2);
            gamma_pid_ki = params(3);
            gamma_pid_kd = params(4);
            delta_t = params(5);
            initial_gamma = params(6);
            min_gamma = params(7);
            max_gamma = params(8);
            max_aoa_target = params(9);
            elevator_pid_kp = params(10);
            elevator_pid_ki = params(11);
            elevator_pid_kd = params(12);
            min_elevator = params(13);
            max_elevator = params(14);
            initial_airspeed = params(15);
            initial_elevator=0;
            max_airspeed=230;
            
            obj.AltitudeErrorRangeLimiter = rangeLimiter(...
                -max_altitude_error, max_altitude_error);
            obj.GammaPID = pidControler(gamma_pid_kp, gamma_pid_ki, ...
                gamma_pid_kd, delta_t, initial_gamma, ...
                0, min_gamma, max_gamma);
            obj.GammaRangeLimiter = rangeLimiter(min_gamma, max_gamma);
            obj.AoaRangeLimiter = rangeLimiter(-max_aoa_target, ...
                max_aoa_target);
            gamma_controler_params = [elevator_pid_kp, elevator_pid_ki, elevator_pid_kd, delta_t, initial_elevator, initial_gamma, min_elevator, max_elevator];
            obj.GammaControler = flightPathControler(gamma_controler_params);
            obj.LastTargetGamma = initial_gamma;
        end
        
        function [target_gamma, u_elevator] = step(obj, ...
                delta_h, h_d, gamma, gamma_d, alpha, airspeed)
            % Method to obtain the command of the elevator using the
            % following parameters :
            %   - delta_h : altitude error in m
            %   - h_d : derivative of the altitude
            %   - gamma : instant flight path in rad
            %   - gamma_d : derivative of the instant flight path in rad/s
            %   - alpha : instant Angle of Attack in rad
            %   - airspeed : airspeed in m/s
            % Returns :
            %   - target_gamma : target flight path in rad
            %   - u_elevator : elevator command in rad
            % The description of the controler can be found in the control diagram files

            delta_h = obj.AltitudeErrorRangeLimiter.step(delta_h);
            gamma_pid_output = obj.GammaPID.step(delta_h, -h_d);
            limited_gamma_target = obj.GammaRangeLimiter.step(gamma_pid_output);
            aoa_target = obj.AoaRangeLimiter.step(alpha + limited_gamma_target - gamma);
            target_gamma = gamma + aoa_target - alpha;
            gamma_error = target_gamma - gamma;
            gamma_error_d = (target_gamma - obj.LastTargetGamma)/obj.GammaPID.delta_t - gamma_d;
            u_elevator = obj.GammaControler.step(gamma_error, gamma_error_d, airspeed);
            obj.LastTargetGamma = target_gamma;
        end
    end
end

