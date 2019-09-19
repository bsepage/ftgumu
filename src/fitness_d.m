function output = fitness_d( u, k, pbm, cst, agt )
% This function is the fitness function for the genetic algorithm. It aims
% at finding a minimum for the standoff distance.

fct_d = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);

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
output = mean([fct_d(s1Fwd,oFwd) fct_d(s2Fwd,oFwd)],2);
end

