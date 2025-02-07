classdef pidControler < handle
    % Discrete time PID controler
    properties
        kp;
        ki;
        kd;
        delta_t;
        min_integterm=0;
        max_integterm=0;
        integterm=0;
        integterm_isbounded=false;
    end
    methods
        function obj = pidControler(varargin)
        % Construct the PID.
        % arguments :
        %   - kp
        %   - ki
        %   - kd
        %   - delta_t : period at which the controler is called
        %   - initial_output
        %   - initial_input
        %   - [OPTIONAL] min_output_value
        %   - [OPTIONAL] max_output_value
        % The min and max output values are only used to limit the integral term
            obj.kp = varargin{1};
            obj.ki = varargin{2};
            obj.kd = varargin{3};
            obj.delta_t = varargin{4};
            obj.setInitialOutputValue(varargin{5}, varargin{6});
            if nargin == 8
                min_output_value = varargin{7};
                max_output_value = varargin{8};
                obj.setIntegtermsLimits(min_output_value, max_output_value);
            elseif nargin ~= 6
                error('Incorrect number of arguments');
            end
        end
        
        function setInitialOutputValue(obj, output_value, input_value)
            % Sets the integral term to the right value to have the wanted
            % output value for an initial input value of inputValue. If
            % ki=0, the method does nothing and the initial output is not
            % controled.
            if obj.ki ~= 0
                obj.integterm = (output_value - obj.kp * input_value - obj.ki * obj.delta_t * input_value) / obj.ki;
            else
                warning('It is not possible to set en initial output value for ki=0')
            end
        end
        
        function setIntegtermsLimits(obj, min_out_value, max_out_value)
            % Sets the min and max values of the output
            if obj.ki == 0
                warning('It is not possible to set limits for the integral term if ki=0')
            else
                obj.min_integterm = min_out_value / obj.ki;
                obj.max_integterm = max_out_value / obj.ki;
                obj.integterm_isbounded = true;
            end
        end
        
        function y = step(obj, x, x_d)
            % Method to obtain the control command using the following parameters :
            %   - x : error between the desired value and the current value
            %   - x_d : derivative of the error between the desired value and the current value
            % Returns the control command. This method has to be called once at each iteration
            obj.integterm = obj.integterm + x*obj.delta_t;
            if obj.integterm < obj.min_integterm && obj.integterm_isbounded
                obj.integterm = obj.min_integterm;
            elseif obj.integterm > obj.max_integterm && obj.integterm_isbounded
                obj.integterm = obj.max_integterm;
            end
            y = obj.kp * x + obj.ki * obj.integterm + obj.kd * x_d;
        end
    end
end