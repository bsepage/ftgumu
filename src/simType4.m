% Author: Benjamin Semal
% Date: 02/05/2016
% @: Cranfield University


%% INITIALISATION
pbm.sol = 'independant';

for k=1:cst.N-1,
    agt.TGT1.o(:,k+1) = [agt.TGT1.o(1,k) + cst.Ts*agt.TGT1.v;
                         agt.TGT1.o(2,k);
                         agt.TGT1.o(3,k) - cst.std_h0+2*cst.std_h0*rand(1)];
    agt.TGT2.o(:,k+1) = [agt.TGT2.o(1,k) + cst.Ts*agt.TGT2.v;
                         agt.TGT2.o(2,k);
                         agt.TGT2.o(3,k) - cst.std_h0+2*cst.std_h0*rand(1)];
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

