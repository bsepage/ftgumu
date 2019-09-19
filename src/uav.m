classdef uav
    properties
        d           % Relative distance
        D           % Position offset
        gamma       % Separation angle
        L           % Trace(P)
        P           % Geolocation error covariance
        psi         % Heading
        s           % Position
        theta       % Relative angle
        v           % Speed
        color
        marker
        trace
    end
    
    methods
        function obj = uav(N,s_start,psi,color,marker,trace,v,D)
            obj.d       = zeros(1,N-1);
            obj.D       = D;
            obj.gamma   = zeros(1,N-1);
            obj.L       = zeros(1,N-1);
            obj.P       = zeros(2,2,N-1);
            obj.psi     = zeros(1,N);
            obj.psi(1)  = psi;
            obj.s       = zeros(3,N);
            obj.s(:,1)  = s_start;
            obj.s(3,:)  = obj.s(3,1);
            obj.theta   = zeros(1,N-1);
            obj.v       = v;
            obj.color   = color;
            obj.marker  = marker;
            obj.trace   = trace;
        end
    end    
end