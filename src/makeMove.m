function agt = makeMove( k, pbm, cst, data, agt )
    global gridP gridX
    fct_d   = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);
    
    if strcmp(pbm.sol,'independant'),
        if pbm.nUAV==1, UAVs = agt.UAV1;
        else UAVs = [agt.UAV1 agt.UAV2];
        end
        for i=1:pbm.nUAV,
            % Current state-space
            sCrt = UAVs(i).s(:,k);
            sCrt = sCrt + [-cst.std_s+2*cst.std_s*rand(1);
                           -cst.std_s+2*cst.std_s*rand(1);
                           0];
            if pbm.scn==3, oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
            else oCrt = agt.TGT1.o(:,k); end
            xCrt = [oCrt(1)-sCrt(1);oCrt(2)-sCrt(2);UAVs(i).psi(k)];
            xCrt(3) = xCrt(3) - cst.std_th+2*cst.std_th*rand(1);
            xCrt = [round(xCrt(1)/cst.q)*cst.q;
                    round(xCrt(2)/cst.q)*cst.q;
                    round(xCrt(3)/0.5)*0.5];
            % Index of current state-space
            i1 = find(data.X1==xCrt(1));
            i2 = find(data.X2==xCrt(2));
            i3 = find(data.X3==xCrt(3));
            % Optimal forward position
            xFwd = gridX{i,i1,i2,i3,k};
            if pbm.scn==3, oFwd = mean([agt.TGT1.o(:,k+1) agt.TGT2.o(:,k+1)],2);
            else oFwd = agt.TGT1.o(:,k+1); end
            sFwd = [oFwd(1)-xFwd(1);oFwd(2)-xFwd(2);cst.zUAV];
            % Write forward position and heading
            UAVs(i).s(:,k+1) = sFwd;
            UAVs(i).psi(k+1) = xFwd(3);
            % Write trace(P), cost, and planar distance
            UAVs(i).P(:,:,k) = gridP{i1,i2,i3,k};
            UAVs(i).L(k) = trace(UAVs(i).P(:,:,k));
            UAVs(i).d(k) = fct_d(UAVs(i).s(:,k),agt.TGT1.o(:,k));
            UAVs(i).theta(k) = get_theta(oCrt,sCrt);
        end
        if pbm.nUAV==1,
            UAVs(1).gamma(k) = UAVs(1).theta(k);
            agt.UAV1 = UAVs(1);
        else
            agt.UAV1 = UAVs(1);
            agt.UAV2 = UAVs(2);
            agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
            agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
        end
        
    elseif strcmp(pbm.sol,'dependant'),
        % Current state-space
        sCrt = agt.UAV2.s(:,k);
        sCrt = sCrt + [-cst.std_s+2*cst.std_s*rand(1);
                       -cst.std_s+2*cst.std_s*rand(1);
                       0];
        if pbm.scn==3, oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
        else oCrt = agt.TGT1.o(:,k); end
        xCrt = [oCrt(1)-sCrt(1);oCrt(2)-sCrt(2);agt.UAV2.psi(k)];
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
        agt.UAV2.s(:,k+1) = sFwd;
        agt.UAV2.psi(k+1) = xFwd(3);
        % Write trace(P), cost, and planar distance
        agt.UAV2.P(:,:,k) = gridP{i1,i2,i3,k};
        agt.UAV2.L(k) = trace(agt.UAV2.P(:,:,k));
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),agt.TGT1.o(:,k));
        agt.UAV2.theta(k) = get_theta(oCrt,sCrt);
        agt.UAV2.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        
    elseif strcmp(pbm.sol,'group'),
        % Current state-space
        sCrt = mean([agt.UAV1.s(:,k) agt.UAV2.s(:,k)],2);
        sCrt = sCrt + [-cst.std_s+2*cst.std_s*rand(1);
                       -cst.std_s+2*cst.std_s*rand(1);
                       0];
        if pbm.scn==3, oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
        else oCrt = agt.TGT1.o(:,k); end
        xCrt = [oCrt(1)-sCrt(1);oCrt(2)-sCrt(2);agt.UAV1.psi(k)];
        xCrt(3) = xCrt(3) - cst.std_th+2*cst.std_th*rand(1);
        xCrt = [round(xCrt(1)/cst.q)*cst.q;
                round(xCrt(2)/cst.q)*cst.q;
                round(xCrt(3)/0.5)*0.5];
        % Index of current state-space
        i1 = find(data.X1==xCrt(1));
        i2 = find(data.X2==xCrt(2));
        i3 = find(data.X3==xCrt(3));
        % Optimal forward position
        xFwd = gridX{1,i1,i2,i3,k};
        if pbm.scn==3, oFwd = mean([agt.TGT1.o(:,k+1) agt.TGT2.o(:,k+1)],2);
        else oFwd = agt.TGT1.o(:,k+1); end
        sFwd = [oFwd(1)-xFwd(1);oFwd(2)-xFwd(2);cst.zUAV];
        % Write forward position and heading
        agt.UAV1.s(:,k+1) = sFwd + agt.UAV1.D;
        agt.UAV2.s(:,k+1) = sFwd + agt.UAV2.D;
        agt.UAV1.s(3,k+1) = agt.UAV1.s(3,k);
        agt.UAV2.s(3,k+1) = agt.UAV2.s(3,k);
        agt.UAV1.psi(k+1) = xFwd(3);
        agt.UAV2.psi(k+1) = xFwd(3);
        % Write trace(P), cost, and planar distance
        agt.UAV1.P(:,:,k) = gridP{i1,i2,i3,k};
        agt.UAV2.P(:,:,k) = gridP{i1,i2,i3,k};
        agt.UAV1.L(k) = trace(agt.UAV1.P(:,:,k));
        agt.UAV2.L(k) = trace(agt.UAV2.P(:,:,k));
        agt.UAV1.d(k) = fct_d(agt.UAV1.s(:,k),agt.TGT1.o(:,k));
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),agt.TGT1.o(:,k));
        agt.UAV1.theta(k) = get_theta(oCrt,sCrt+agt.UAV1.D);
        agt.UAV2.theta(k) = get_theta(oCrt,sCrt+agt.UAV2.D);
        agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
    end
end

