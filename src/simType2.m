% Author: Benjamin Semal
% Date: 02/05/2016
% @: Cranfield University


%% INITIALISATION
fct_psi = @(psi,u,Ts) mod(psi+u*Ts,2*pi);
fct_d   = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);

if strcmp(pbm.tec,'DP'),
    if strcmp(pbm.sol,'dependant'),
        simType1
        display('simType2')
        pbm.sol = 'dependant';
    end
    display(pbm)
elseif strcmp(pbm.tec,'greedy'),
    if ~strcmp(pbm.sol,'independant'),
        error('The greedy technique works ONLY with the independant solution')
    end
    display(pbm)
elseif strcmp(pbm.tec,'genetic'),
    if strcmp(pbm.sol,'dependant'),
        error('The genetic technique does not work with the dependant solution')
    end
    display(pbm)
elseif strcmp(pbm.tec,'PSO'),
    if strcmp(pbm.sol,'dependant'),
        error('The PSO technique does not work with the dependant solution choice')
    end
    display(pbm)
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
    
elseif strcmp(pbm.tec,'greedy'),
    for n=1:cst.N-1,
        clc
        display(pbm)
        display(n)
        % Run DP independently
        display('Running DP round 1/2')
        pbm.sol = 'independant';
        for k=cst.N-1:-1:n,
            display(k)
            tstart = tic;
            DP(k,pbm,cst,data,agt)
            display(toc(tstart))
        end
        % Make a move
        display('Moving reference UAV')
        for k=n:cst.N-1,
            agt = makeMove(k,pbm,cst,data,agt);
        end
        % Election of the reference UAV
        if agt.UAV1.L(n) <= agt.UAV2.L(n),
            UAV_ref = agt.UAV1;
            indexSlave = 2;
        else
            UAV_ref = agt.UAV2;
            indexSlave = 1;
        end
        % Run DP w.r.t. the reference UAV
        display('Running DP round 2/2')
        for k=cst.N-1:-1:n,
            display(k)
            tstart = tic;
            greedy(k,pbm,cst,data,agt,UAV_ref)
            display(toc(tstart))
        end
        % Make a move with the slave UAV
        display('Moving slave UAV')
        for k=n:cst.N-1,
            agt = makeGreedyMove(k,pbm,cst,data,agt,indexSlave);
        end
    end
    
elseif strcmp(pbm.tec,'genetic'),
    options = gaoptimset('PopulationSize',50);
    for k=1:cst.N-1,
        display(k)
        tstart = tic;
        % Set fitness function & nonlinear constraints
        if strcmp(pbm.fitness,'P'), fitness = @(u) fitnessP(u,k,pbm,cst,agt);
        elseif strcmp(pbm.fitness,'d'), fitness = @(u) fitness_d(u,k,pbm,cst,agt);
        elseif strcmp(pbm.fitness,'gamma'), fitness = @(u) fitness_gamma(u,k,pbm,cst,agt);
        else error('The fitness parameter is not set properly');
        end
        nonlcon = @(u) setNonLCon(u,k,pbm,cst,agt);
        % Update d & gamma
        agt.UAV1.theta(k) = get_theta(agt.TGT1.o(:,k),agt.UAV1.s(:,k));
        agt.UAV2.theta(k) = get_theta(agt.TGT1.o(:,k),agt.UAV2.s(:,k));
        agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
        agt.UAV1.d(k) = fct_d(agt.UAV1.s(:,k),agt.TGT1.o(:,k));
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),agt.TGT1.o(:,k));
        if strcmp(pbm.sol,'independant'),
            % Update P
            P1 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P2 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            agt.UAV1.P(:,:,k) = P1;
            agt.UAV2.P(:,:,k) = P2;
            agt.UAV1.L(k) = trace(P1);
            agt.UAV2.L(k) = trace(P2);
            % Run GA algorithm
            [optU,min_L] = ga(fitness,2,[],[],[],[],[-0.5;-0.5],[0.5;0.5],[],options);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU(1),cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU(2),cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU(1),cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU(2),cst.Ts);
        elseif strcmp(pbm.sol,'group'),
            % Update P
            P1 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P2 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P = inv(inv(P1)+inv(P2));
            agt.UAV1.P(:,:,k) = P;
            agt.UAV2.P(:,:,k) = P;
            agt.UAV1.L(k) = trace(P);
            agt.UAV2.L(k) = trace(P);
            % Run GA algorithm
            [optU,min_L] = ga(fitness,1,[],[],[],[],-0.5,0.5,[],options);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU,cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU,cst.Ts);
        end
        display(toc(tstart))
    end
    
elseif strcmp(pbm.tec,'PSO'),
    for k=1:cst.N-1,
        display(k)
        tstart = tic;
        % Set fitness function & nonlinear constraints
        if strcmp(pbm.fitness,'P'), fitness = @(u) fitnessP(u,k,pbm,cst,agt);
        elseif strcmp(pbm.fitness,'d'), fitness = @(u) fitness_d(u,k,pbm,cst,agt);
        elseif strcmp(pbm.fitness,'gamma'), fitness = @(u) fitness_gamma(u,k,pbm,cst,agt);
        else error('The fitness parameter is not set properly');
        end
        % Update d & gamma
        agt.UAV1.theta(k) = get_theta(agt.TGT1.o(:,k),agt.UAV1.s(:,k));
        agt.UAV2.theta(k) = get_theta(agt.TGT1.o(:,k),agt.UAV2.s(:,k));
        agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
        agt.UAV1.d(k) = fct_d(agt.UAV1.s(:,k),agt.TGT1.o(:,k));
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),agt.TGT1.o(:,k));
         % Find u* and make a move
        if strcmp(pbm.sol,'independant'),
            % Update P
            P1 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P2 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            agt.UAV1.P(:,:,k) = P1;
            agt.UAV2.P(:,:,k) = P2;
            agt.UAV1.L(k) = trace(P1);
            agt.UAV2.L(k) = trace(P2);
            % Run PSO algorithm
            [optU,min_L] = particleswarm(fitness,2,[-0.5;-0.5],[0.5;0.5]);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU(1),cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU(2),cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU(1),cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU(2),cst.Ts);
        elseif strcmp(pbm.sol,'group'),
            % Update P
            P1 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P2 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P = inv(inv(P1)+inv(P2));
            agt.UAV1.P(:,:,k) = P;
            agt.UAV2.P(:,:,k) = P;
            agt.UAV1.L(k) = trace(P);
            agt.UAV2.L(k) = trace(P);
            % Run PSO algorithm
            [optU,min_L] = particleswarm(fitness,1,-0.5,0.5);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU,cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU,cst.Ts);
        end
        display(toc(tstart))
    end
end

