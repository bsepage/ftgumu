function output = getP( o, s, psi, std_th, std_s, std_h0 )
% This function performs all the steps to compute the geolocation error
% covariance P

fct_uS  = @(q) q/sqrt(q.'*q);                   % LOS vector in S frame
fct_uT  = @(T,uS) T\uS;                         % LOS vector in T frame
fct_r   = @(o,s,uT) (o(3)-s(3))/uT(3);          % Range UAV/target
fct_Bs  = @(uT) eye(3)-(1/uT(3))*uT*[0 0 1];    % Components to compute P
fct_Bu  = @(o,s,r,uT) r*eye(3)+((s(3)-o(3))/(uT(3))^2)*uT*[0 0 1];
fct_P1  = @(Bs,R_s) Bs*R_s*Bs';
fct_P2 	= @(uT,std_h0) (1/uT(3))^2*uT*(uT).'*std_h0^2;
fct_P3 	= @(Bu,C,R_th) Bu*C.'*R_th*C*Bu.';

R_th    = eye(3)*std_th^2;              % Sensor attitude error covariance
R_s     = eye(3)*std_s^2;               % Sensor location error covariance
theta   = [psi;0;0];                    % Sensor attitude
T       = getT(theta);                  % Sensor attitude matrix
uS      = fct_uS(get_q(s,o,T));         % LOS vector in the S frame
uT      = fct_uT(T,uS);                 % LOS vector in the T frame
A       = [1 0 0;0 1 0];                % Weighting matrix
output = A*(fct_P1(fct_Bs(uT),R_s)+...  % Computation of P (2x2)
            fct_P2(uT,std_h0)+...
            fct_P3(fct_Bu(o,s,fct_r(o,s,uT),uT),getC(theta,uS),R_th))*A';
end

