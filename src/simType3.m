% Author: Benjamin Semal
% Date: 02/05/2016
% @: Cranfield University


%% INITIALISATION
fct_psi = @(psi,u,Ts) mod(psi+u*Ts,2*pi);
fct_d   = @(s,o) sqrt((s(1)-o(1))^2+(s(2)-o(2))^2);

if strcmp(pbm.tec,'DP'),
    if strcmp(pbm.sol,'dependant'),
        simType4
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
        display('Running DP dependently')
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
        oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
        agt.UAV1.theta(k) = get_theta(oCrt,agt.UAV1.s(:,k));
        agt.UAV2.theta(k) = get_theta(oCrt,agt.UAV2.s(:,k));
        agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
        agt.UAV1.d(k) = fct_d(agt.UAV1.s(:,k),oCrt);
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),oCrt);
        if strcmp(pbm.sol,'independant'),
            % Update P
            P11 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P12 = getP(agt.TGT2.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P21 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P22 = getP(agt.TGT2.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            agt.UAV1.P(:,:,k) = P11+P12;
            agt.UAV2.P(:,:,k) = P21+P22;
            agt.UAV1.L(k) = trace(P11+P12);
            agt.UAV2.L(k) = trace(P21+P22);
            % Run GA algorithm
            [optU,min_L] = ga(fitness,2,[],[],[],[],[-0.5;-0.5],[0.5;0.5],[],options);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU(1),cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU(2),cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU(1),cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU(2),cst.Ts);
        elseif strcmp(pbm.sol,'group'),
            P11 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P12 = getP(agt.TGT2.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P21 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P22 = getP(agt.TGT2.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
            P = inv(inv(P11+P12)+inv(P21+P22));
            agt.UAV1.P(:,:,k) = P;
            agt.UAV2.P(:,:,k) = P;
            agt.UAV1.L(k) = trace(P);
            agt.UAV2.L(k) = trace(P);
            % Run GA algorithm
            [optU,min_d] = ga(fitness,1,[],[],[],[],-0.5,0.5,[],options);
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
        % Update of P, d & gamma
        P11 = getP(agt.TGT1.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
        P12 = getP(agt.TGT2.o(:,k),agt.UAV1.s(:,k),agt.UAV1.psi(k),cst.std_th,cst.std_s,cst.std_h0);
        P21 = getP(agt.TGT1.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
        P22 = getP(agt.TGT2.o(:,k),agt.UAV2.s(:,k),agt.UAV2.psi(k),cst.std_th,cst.std_s,cst.std_h0);
        P = inv(inv(P11+P12)+inv(P21+P22));
        agt.UAV1.P(:,:,k) = P;
        agt.UAV2.P(:,:,k) = P;
        agt.UAV1.L(k) = trace(P);
        agt.UAV2.L(k) = trace(P);
        oCrt = mean([agt.TGT1.o(:,k) agt.TGT2.o(:,k)],2);
        agt.UAV1.theta(k) = get_theta(oCrt,agt.UAV1.s(:,k));
        agt.UAV2.theta(k) = get_theta(oCrt,agt.UAV2.s(:,k));
        agt.UAV1.gamma(k) = abs(agt.UAV1.theta(k)-agt.UAV2.theta(k));
        agt.UAV2.gamma(k) = agt.UAV1.gamma(k);
        agt.UAV1.d(k) = fct_d(agt.UAV1.s(:,k),oCrt);
        agt.UAV2.d(k) = fct_d(agt.UAV2.s(:,k),oCrt);
        if strcmp(pbm.sol,'independant'),
            % Run PSO algorithm
            [optU,min_L] = particleswarm(fitness,2,[-0.5;-0.5],[0.5;0.5]);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU(1),cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU(2),cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU(1),cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU(2),cst.Ts);
        elseif strcmp(pbm.sol,'group'),
            % Run PSO algorithm
            [optU,min_d] = particleswarm(fitness,1,-0.5,0.5);
            % Make a move
            agt.UAV1.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV1.psi(:,k),agt.UAV1.s(:,k));
            agt.UAV2.s(:,k+1)   = get_s(cst.vUAV,optU,cst.Ts,agt.UAV2.psi(:,k),agt.UAV2.s(:,k));
            agt.UAV1.psi(:,k+1) = fct_psi(agt.UAV1.psi(:,k),optU,cst.Ts);
            agt.UAV2.psi(:,k+1) = fct_psi(agt.UAV2.psi(:,k),optU,cst.Ts);
        end
        display(toc(tstart))
    end
end

