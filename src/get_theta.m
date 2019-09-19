function output = get_theta( o, s )
% This function computes the relative angle between the sensor s and its
% target o

dx = s(1)-o(1);
dy = s(2)-o(2);    
if dx~=0, output = atan(dy/dx);
elseif dx==0 && dy>0, output = pi/2;
elseif dx==0 && dy<0, output = -pi/2;
else output = 0;
end
end

