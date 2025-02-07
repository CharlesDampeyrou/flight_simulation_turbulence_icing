classdef gearControler < handle
    % Class to control the gear of the plane. The gear is extended if the altitude is below the limit
    properties
        AltitudeLimit
    end
    
    methods
        function obj = gearControler(params)
            obj.AltitudeLimit = params(1);
        end
        
        function u_gear = step(obj,h)
            % This method gives the gear command from the plane's altitude
            if h<obj.AltitudeLimit
                u_gear = 1;
            else
                u_gear = 0;
            end
        end
    end
end

