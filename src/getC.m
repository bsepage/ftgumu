function output = getC( t, uS)
% This function computes the linearised sensor attitude matrix

persistent t1 t2 t3 u1 u2 u3
t1 = t(1);
t2 = t(2);
t3 = t(3);
u1 = uS(1);
u2 = uS(2);
u3 = uS(3);
c1 = [-cos(t2)*sin(t1)*u1-(cos(t3)*cos(t1)+sin(t3)*sin(t2)*sin(t1))*u2+(sin(t3)*cos(t1)-cos(t3)*sin(t2)*sin(t1))*u3;
      -sin(t2)*cos(t1)*u1+sin(t3)*cos(t2)*cos(t1)*u2+cos(t3)*cos(t2)*cos(t1)*u3;
      (sin(t3)*sin(t1)+cos(t3)*sin(t2)*cos(t1))*u2+(cos(t3)*sin(t1)-sin(t3)*sin(t2)*cos(t1))*u3];
c2 = [cos(t2)*cos(t1)*u1+(-cos(t3)*sin(t1)+sin(t3)*sin(t2)*cos(t1))*u2+(sin(t3)*sin(t1)+cos(t3)*sin(t2)*cos(t1))*u3;
      -sin(t2)*cos(t1)*u1+sin(t3)*cos(t2)*sin(t1)*u2+cos(t3)*sin(t2)*sin(t1)*u3;
      (-sin(t3)*cos(t1)*u1+cos(t3)*sin(t2)*sin(t1))*u2-(cos(t3)*cos(t1)+sin(t3)*sin(t2)*sin(t1))*u3];
c3 = [0;
      -cos(t2)*u1 + sin(t3)*sin(t2)*u2 - cos(t3)*sin(t2)*u3;
      cos(t3)*cos(t2)*u2 - sin(t3)*cos(t2)*u3];
output = [c1 c2 c3];
end

