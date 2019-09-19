function output = fitness_gamma( u, k, pbm, cst, agt )
% This function is the fitness function for the genetic algorithm. It aims
% at finding a minimum for the separation angle.

% UAVs forward position
if strcmp(pbm.sol,'group'),
    s1Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV1.psi(k),agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
else
    s1Fwd = get_s(cst.vUAV,u(1),cst.Ts,agt.UAV1.psi(k),agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u(2),cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
end
% Target forward position
if pbm.nTGT==2,
    o1Fwd = agt.TGT1.o(:,k+1);
    o2Fwd = agt.TGT2.o(:,k+1);
    oFwd = mean([o1Fwd o2Fwd],2);
else
    oFwd = agt.TGT1.o(:,k+1);
end
% UAVs relative angle to the target
theta1 = get_theta(oFwd,s1Fwd);
theta2 = get_theta(oFwd,s2Fwd);
output = abs(cos(abs(theta1-theta2)));
end

