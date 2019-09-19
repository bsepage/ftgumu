function output = get_s( v, u, Ts, psi, s )
% This function the UAV position in the T frame

% Zero input
if u==0,
    s(1) = s(1) + v*Ts*cos(psi);
    s(2) = s(2) + v*Ts*sin(psi);
% Non-zero input
else
    s(1) = s(1) + v/u*(sin(u*Ts+psi)-sin(psi));
    s(2) = s(2) + v/u*(cos(psi)-cos(u*Ts+psi));
end
output = [s(1);s(2);s(3)];
end

