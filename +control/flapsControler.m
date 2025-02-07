classdef flapsControler < handle
    % Class to control the flaps of the plane. If the airspeed is below the limit or the altitude is below the limit, the flaps are extended. Otherwise, the flaps are set to the desired angle
    
    properties
        AirspeedLimit
        AltitudeLimit
    end
    
    methods
        function obj = flapsControler(params)
            % params = [
            % air_speed_lim : airspeed bellow which the flaps are extended, m/s,
            % altitude_lim : altitude bellow which the flaps are extended, m,
            %]
            obj.AirspeedLimit = params(1);
            obj.AltitudeLimit = params(2);
        end
        
        function u_flaps = step(obj,flap_instruction, tas, h)
            % Method to obtain the flap command using the following parameters :
            %   - flap_instruction : desired flap angle in rad
            %   - tas : true airspeed in m/s
            %   - h : altitude in m
            % Returns the flap command in rad
            % If the airspeed is below the limit or the altitude is below the limit, the flaps are set to 38 degrees
            % Otherwise, the flaps are set to the desired angle
            if (tas<obj.AirspeedLimit || h<obj.AltitudeLimit)
                u_flaps = 38 * pi/180;
            else
                u_flaps = flap_instruction;
            end
        end
    end
end

