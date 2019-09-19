classdef target
    properties
        o           % Position
        v           % Speed
        color
        marker
        trace
    end
    
    methods
        function obj = target(N,o_start,speed,color,marker,trace)
            obj.o       = zeros(3,N);
            obj.o(:,1)  = o_start;
            obj.v       = speed;
            obj.color   = color;
            obj.marker  = marker;
            obj.trace   = trace;
        end
    end    
end

