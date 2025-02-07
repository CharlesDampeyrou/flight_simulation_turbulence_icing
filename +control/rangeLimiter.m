classdef rangeLimiter < handle
    % Class limiting the range of a command
    properties
        Min
        Max
    end
    
    methods
        function obj = rangeLimiter(min, max)
            % Parameters :
            %   - min : the minimum value of the command
            %   - max : the maximum value of the command
            obj.Min = min;
            obj.Max = max;
        end
        
        function y = step(obj,input)
            % Method to limit the range of the command
            if input < obj.Min
                y = obj.Min;
            elseif input > obj.Max
                y = obj.Max;
            else
                y = input;
            end
        end
    end
end

