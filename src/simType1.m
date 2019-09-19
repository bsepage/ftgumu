% Author: Benjamin Semal
% Date: 02/05/2016
% @: Cranfield University


%% INITIALISATION
if pbm.scn==1,
    if ~strcmp(pbm.tec,'DP'),
        error('This scenario computes only the DP technique')
    end
    if ~strcmp(pbm.sol,'independant'),
        error('This scenario computes only the independant solution')
    end
    display(pbm)
elseif pbm.scn==2,
    pbm.sol = 'independant';
    display('simType1')
end

for k=1:cst.N-1,
    agt.TGT1.o(:,k+1) = [agt.TGT1.o(1,k) + cst.Ts*agt.TGT1.v;
                         agt.TGT1.o(2,k);
                         agt.TGT1.o(3,k) - cst.std_h0+2*cst.std_h0*rand(1)];
end


%% COMPUTATION
if strcmp(pbm.tec,'DP'),
    % Running DP
    for k=cst.N-1:-1:1,
        display(k)
        tstart = tic;
        DP(k,pbm,cst,data,agt)
        display(toc(tstart))
    end
    % Apply optimal trajectory
    for k=1:cst.N-1,
        agt = makeMove(k,pbm,cst,data,agt);
    end
end

