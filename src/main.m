% Author: Benjamin Semal
% Date: 24/08/2016
% @: Cranfield University
%
% Descirption: This script is the main script. It allows setting the pbm
% structure, generates all variables (constants.m), run the selected
% algorithm, generate the plots (getPlots.m), and clean the workspace

clc; clear all; close all; format compact; format short;


%% Settings (see Table 2)
% - scenario  = 1, 2, 3
% - technique = DP, greedy, genetic, PSO
% - solution  = independant, dependant, group
scenario    = 2;
technique   = 'PSO';
solution    = 'independant';
fitness     = 'd';

constants;


%% Run algorithm
tstartSim = tic;
for m=1:cst.N_MC,
    display(m)
    % Randomise agents initial location
%     agt.TGT1.o(2,1) = -30+60*rand(1);
%     agt.UAV1.s(2,1) = -30+60*rand(1);
%     if pbm.nTGT==2, agt.TGT2.o(2,1) = -15+30*rand(1); end
%     if pbm.nUAV==2,
%         if strcmp(pbm.sol,'group'), agt.UAV2.s(:,1) = agt.UAV1.s(:,1) + 2*agt.UAV2.D;
%         else agt.UAV2.s(2,1) = -15+30*rand(1); end
%     end
    % Run algorithm
    switch(pbm.scn)
        case 1
            simType1
        case 2
            simType2
        case 3
            simType3
        otherwise
            error('The scenario is not set properly')
    end
    % Store average L for Monte-Carlo plots
    if pbm.nUAV==1, data.outputVector(m) = mean(agt.UAV1.L);
    else data.outputVector(m) = (mean(agt.UAV1.L)+mean(agt.UAV2.L))/2; end
    clc
end
display(toc(tstartSim))


%% Plots
getPlots;


%% Clean workspace
clearvars -except pbm cst data agt

