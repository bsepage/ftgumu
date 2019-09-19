function output = get_q( s, o, T )
% This function computes the normalised image point coordinates

persistent rx ry rz
rx = o(1) - s(1);
ry = o(2) - s(2);
rz = o(3) - s(3);
xn = (T(1,1)*rx+T(1,2)*ry+T(1,3)*rz)/(T(3,1)*rx+T(3,2)*ry+T(3,3)*rz);
yn = (T(2,1)*rx+T(2,2)*ry+T(2,3)*rz)/(T(3,1)*rx+T(3,2)*ry+T(3,3)*rz);
zn = 1;
output = [xn;yn;zn];
end

