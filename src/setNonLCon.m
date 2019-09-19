function [ c, c_eq ] = setNonLCon( u, k, pbm, cst, agt )
% This function sets nonlinear constraints in the search of the minimum
% input for the fitness function.

fct_d = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);

if pbm.nTGT==2,
    o1Fwd = agt.TGT1.o(:,k+1);
    o2Fwd = agt.TGT2.o(:,k+1);
    oFwd = mean([o1Fwd o2Fwd],2);
else
    oFwd = agt.TGT1.o(:,k+1);
end

if strcmp(pbm.sol,'group'),
    s1Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV1.psi(k), agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
else
    s1Fwd = get_s(cst.vUAV,u(1),cst.Ts,agt.UAV1.psi(k),agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u(2),cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
end

c       = [fct_d(s1Fwd,oFwd)-104 fct_d(s2Fwd,oFwd)-104];
c_eq    = [];
end

