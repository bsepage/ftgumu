function greedy( k, pbm, cst, data, agt, UAV_ref )
    global gridV gridP gridX
    fct_psi = @(psi,u,Ts) mod(psi+u*Ts,2*pi);
    
    for x1=1:data.x1Last,
        for x2=1:data.x2Last,
            for x3=1:data.x3Last,
                gridVCrt = {};
                gridPCrt = {};
                gridXCrt = {};
                admissibleInputs = 0;
                for u=1:data.uLast,
                    % Compute current position
                    if pbm.scn==3, oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
                    else oCrt = agt.TGT1.o(:,k); end
                    sCrt = [oCrt(1)-data.X1(x1);oCrt(2)-data.X2(x2);cst.zUAV];
                    sCrt = sCrt + [-cst.std_s+2*cst.std_s*rand(1);
                                   -cst.std_s+2*cst.std_s*rand(1);
                                   0];
                    % Compute forward position
                    if pbm.scn==3, oFwd = mean([agt.TGT1.o(:,k+1) agt.TGT2.o(:,k+1)],2);
                    else oFwd = agt.TGT1.o(:,k+1); end
                    sFwd = get_s(cst.vUAV,data.U(u),cst.Ts,data.X3(x3),sCrt);
                    % Compute forward state-space
                    x1Fwd = oFwd(1)-sFwd(1);
                    x2Fwd = oFwd(2)-sFwd(2);
                    x3Fwd = fct_psi(data.X3(x3),data.U(u),cst.Ts);
                    x3Fwd = x3Fwd - cst.std_th+2*cst.std_th*rand(1);
                    % If forward state-space satisfies constraints
                    if abs(x1Fwd)<data.X1(data.x1Last) && abs(x2Fwd)<data.X2(data.x2Last),
                        % Discretize forward state-space
                        xFwd = [round(x1Fwd/cst.q)*cst.q...
                                round(x2Fwd/cst.q)*cst.q...
                                round(x3Fwd/0.5)*0.5];
                        % Find forward state-space index
                        i1 = find(data.X1==xFwd(1));
                        i2 = find(data.X2==xFwd(2));
                        i3 = find(data.X3==xFwd(3));
                        % If forward state accepts an input
                        if isempty(gridV{i1,i2,i3,k+1})==0,
                            % Compute cost
                            if pbm.scn==3,
                                P1 = UAV_ref.P(:,:,k);
                                P21 = getP(agt.TGT1.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
                                P22 = getP(agt.TGT2.o(:,k+1),sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
                                P = inv(inv(P1)+inv(P21+P22));
                            else
                                P1 = UAV_ref.P(:,:,k);
                                P2 = getP(oFwd,sFwd,xFwd(3),cst.std_th,cst.std_s,cst.std_h0);
                                P = inv(inv(P1)+inv(P2));
                            end
                            optFwdV = gridV{i1,i2,i3,k+1};
                            V = optFwdV + trace(P);
                            admissibleInputs = admissibleInputs+1;
                            gridVCrt{admissibleInputs} = V;
                            gridPCrt{admissibleInputs} = P;
                            gridXCrt{admissibleInputs} = xFwd;
                        end
                    end
                end
                % If current state-space admits an input
                if isempty(gridVCrt)==0,
                    % Get optimal V, P, xFwd
                    matVCrt = cell2mat(gridVCrt);
                    optIndex = find(matVCrt==min(matVCrt));
                    gridV{x1,x2,x3,k} = gridVCrt{optIndex};
                    gridP{x1,x2,x3,k} = gridPCrt{optIndex};
                    gridX{1,x1,x2,x3,k} = gridXCrt{optIndex};
                    gridX{2,x1,x2,x3,k} = gridXCrt{optIndex};
                end
            end
        end
    end
end

