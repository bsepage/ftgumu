function output = fitnessP( u, k, pbm, cst, agt )
% This function is the fitness function for the genetic algorithm. It aims
% at finding a minimum for the trace of the error covariance.

fct_psi = @(psi,u,Ts) mod(psi+u*Ts,2*pi);

% UAVs forward position
if strcmp(pbm.sol,'group'),
    s1Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV1.psi(k),agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u,cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
    x13Fwd  = fct_psi(agt.UAV1.psi(k),u,cst.Ts);
	x23Fwd  = fct_psi(agt.UAV2.psi(k),u,cst.Ts);
else
    s1Fwd = get_s(cst.vUAV,u(1),cst.Ts,agt.UAV1.psi(k),agt.UAV1.s(:,k));
    s2Fwd = get_s(cst.vUAV,u(2),cst.Ts,agt.UAV2.psi(k),agt.UAV2.s(:,k));
    x13Fwd  = fct_psi(agt.UAV1.psi(k),u(1),cst.Ts);
	x23Fwd  = fct_psi(agt.UAV2.psi(k),u(2),cst.Ts);
end
% Target forward position
if pbm.nTGT==2,
    o1Fwd = agt.TGT1.o(:,k+1);
    o2Fwd = agt.TGT2.o(:,k+1);
    P11 = getP(o1Fwd,s1Fwd,x13Fwd,cst.std_th,cst.std_s,cst.std_h0);
	P12 = getP(o2Fwd,s1Fwd,x13Fwd,cst.std_th,cst.std_s,cst.std_h0);
	P21 = getP(o1Fwd,s2Fwd,x23Fwd,cst.std_th,cst.std_s,cst.std_h0);
	P22 = getP(o2Fwd,s2Fwd,x23Fwd,cst.std_th,cst.std_s,cst.std_h0);
	P   = inv(inv(P11+P12)+inv(P21+P22));
else
    oFwd = agt.TGT1.o(:,k+1);
    P1 = getP(oFwd,s1Fwd,x13Fwd,cst.std_th,cst.std_s,cst.std_h0);
    P2 = getP(oFwd,s2Fwd,x23Fwd,cst.std_th,cst.std_s,cst.std_h0);
    P = inv(inv(P1)+inv(P2));
end
output = trace(P);
end

