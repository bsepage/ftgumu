function P = getCost( k, pbm, agt, cst, oFwd, sFwd, xFwd )
% This function computes the cost to go rom k to N by computing the
% geolocation error covariance P from the function getP

if strcmp(pbm.sol,'independant'),
    if pbm.nTGT==2,
        P11 = getP(agt.TGT1.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P12 = getP(agt.TGT2.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P = P11 + P12;
    else
        P = getP(oFwd,sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
    end
elseif strcmp(pbm.sol,'dependant'),
    if pbm.nTGT==2,
        P1 = agt.UAV1.P(:,:,k);
        P21 = getP(agt.TGT1.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P22 = getP(agt.TGT2.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P = inv(inv(P1)+inv(P21+P22));
    else
        P1 = agt.UAV1.P(:,:,k);
        P2 = getP(oFwd,sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P = inv(inv(P1)+inv(P2));
    end
elseif strcmp(pbm.sol,'group'),
    if pbm.nTGT==2,
        P11 = getP(agt.TGT1.o(:,k+1),sFwd(:,1),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P12 = getP(agt.TGT2.o(:,k+1),sFwd(:,1),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P21 = getP(agt.TGT1.o(:,k+1),sFwd(:,2),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P22 = getP(agt.TGT2.o(:,k+1),sFwd(:,2),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P = inv(inv(P11+P12)+inv(P21+P22));
    else
        P1 = getP(oFwd,sFwd(:,1),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P2 = getP(oFwd,sFwd(:,2),xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
        P = inv(inv(P1)+inv(P2));
    end
end
end

