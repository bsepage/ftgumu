function agt = makeGreedyMove( k, pbm, cst, data, agt, indexSlave )
    global gridX
    fct_d   = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);
    
    if indexSlave==1,
        UAV_slv = agt.UAV1;
        UAV_mtr = agt.UAV2;
    else
        UAV_slv = agt.UAV2;
        UAV_mtr = agt.UAV1;
    end
    
    % Current state-space
    sCrt = UAV_slv.s(:,k);
    sCrt = sCrt + [-cst.std_s+2*cst.std_s*rand(1);
                   -cst.std_s+2*cst.std_s*rand(1);
                   0];
    if pbm.scn==3, oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
    else oCrt = agt.TGT1.o(:,k); end
    xCrt = [oCrt(1)-sCrt(1);oCrt(2)-sCrt(2);UAV_slv.psi(k)];
    xCrt(3) = xCrt(3) - cst.std_th+2*cst.std_th*rand(1);
    xCrt = [round(xCrt(1)/cst.q)*cst.q;
            round(xCrt(2)/cst.q)*cst.q;
            round(xCrt(3)/0.5)*0.5];
    % Index of current state-space
    i1 = find(data.X1==xCrt(1));
    i2 = find(data.X2==xCrt(2));
    i3 = find(data.X3==xCrt(3));
    % Optimal forward position
    xFwd = gridX{2,i1,i2,i3,k};
    if pbm.scn==3, oFwd = mean([agt.TGT1.o(:,k+1) agt.TGT2.o(:,k+1)],2);
    else oFwd = agt.TGT1.o(:,k+1); end
    sFwd = [oFwd(1)-xFwd(1);oFwd(2)-xFwd(2);cst.zUAV];
    % Write forward position and heading
    UAV_slv.s(:,k+1) = sFwd;
    UAV_slv.psi(k+1) = xFwd(3);
    % Write trace(P), cost, and planar distance
    UAV_slv.P(:,:,k) = getP(oCrt,sCrt,xCrt(3),cst.std_th,cst.std_s,cst.std_h0);
    UAV_slv.L(k) = trace(UAV_slv.P(:,:,k));
    UAV_slv.d(k) = fct_d(UAV_slv.s(:,k),agt.TGT1.o(:,k));
    UAV_slv.theta(k) = get_theta(oCrt,sCrt);
    UAV_slv.gamma(k) = abs(UAV_mtr.theta(k)-UAV_slv.theta(k));
    
    if indexSlave==1, agt.UAV1 = UAV_slv;
    else agt.UAV2 = UAV_slv; end
end

