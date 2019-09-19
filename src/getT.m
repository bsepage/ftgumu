function output = getT( t )
% This function computes the orthogonal attitude matrix  T allowing
% transformation from the T frame to the S frame

persistent t1 t2 t3
t1 = t(1);
t2 = t(2);
t3 = t(3);
T_SN = zeros(3);
T_SN(1,:) = [cos(t2)*cos(t1),cos(t2)*sin(t1),-sin(t2)];
T_SN(2,:) = [-cos(t3)*sin(t1)+sin(t3)*sin(t2)*cos(t1),...
             cos(t3)*cos(t1)+sin(t3)*sin(t2)*sin(t1),...
             sin(t3)*cos(t2)];
T_SN(3,:) = [sin(t3)*sin(t1)+cos(t3)*sin(t2)*cos(t1),...
             -sin(t3)*cos(t1)+cos(t3)*sin(t2)*sin(t1),...
             cos(t3)*cos(t2)];
output = T_SN*[0 1 0;1 0 0;0 0 -1];
end

