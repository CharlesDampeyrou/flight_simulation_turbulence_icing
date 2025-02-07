classdef dummyControler
    % Dummy controler giving 0 whatever is past to it
    
    properties
        Property1
    end
    
    methods
        function obj = dummyControler(varargin)
            
        end
        
        function zero_output = step(obj,varargin)
            % Returns 0
            zero_output = 0;
        end
    end
end

