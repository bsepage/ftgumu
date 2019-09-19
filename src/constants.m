% Author: Benjamin Semal
% Date: 24/08/2016
% @: Cranfield University
%
% Description: This script initialises build the structures and class
% instances in order to run the desired algorithm. Parameters shall be
% tuned under the "cst structure" cell


%% pbm structure
if      scenario==1, nUAV=1; nTGT=1;
elseif  scenario==2, nUAV=2; nTGT=1;
elseif  scenario==3, nUAV=2; nTGT=2;
else    error('The scenario is not set properly')
end

pbm = struct('scn',scenario,'tec',technique,'sol',solution,...
             'fitness',fitness,'nUAV',nUAV,'nTGT',nTGT);

if      strcmp(pbm.tec,'DP'),
elseif  strcmp(pbm.tec,'greedy'),
elseif  strcmp(pbm.tec,'PSO'),
elseif  strcmp(pbm.tec,'genetic'),
else    error('The technique is not set properly');
end

if      strcmp(pbm.sol,'independant'),
elseif  strcmp(pbm.sol,'dependant'),
elseif  strcmp(pbm.sol,'group'),
else    error('The solution is not set properly');
end


%% cst structure --> SETTINGS REQUIRED <--
N_MC    = 1;
N       = 91;
vUAV    = 15;
vTGT    = 5;
q       = 5;
zMax    = 75;
std_th  = deg2rad(5);
std_s   = 0;
std_h0  = 0;

Ts      = 1;
zUAV    = 100;
wMax    = 0.5;

cst = struct('N_MC',N_MC,'N',N,'vUAV',vUAV,'vTGT',vTGT,'q',q,...
             'zMax',zMax,'std_th',std_th,'std_s',std_s,'std_h0',std_h0,...
             'Ts',Ts,'zUAV',zUAV,'wMax',wMax);


%% data structure
global gridV gridP gridX

U  = [-wMax 0 wMax];
uLast = length(U);

i = 1:(1+2*zMax/q);
X1 = -zMax + q*(i-1);
X2 = -zMax + q*(i-1);
x1Last = length(X1);
x2Last = length(X2);

i = 1:13;
X3 = 0.5*(i-1);
x3Last = length(X3);

gridV = cell(x1Last,x2Last,x3Last,N);
gridP = cell(x1Last,x2Last,x3Last,N-1);
gridX = cell(2,x1Last,x2Last,x3Last,N-1);
for i=1:x1Last,
    for j=1:x2Last,
        for k=1:x3Last,
            gridV{i,j,k,N} = 0;
        end
    end
end

outputVector = zeros(1,N_MC);

data = struct('outputVector',outputVector,'U',U,'X1',X1,'X2',X2,'X3',X3,...
              'uLast',uLast,'x1Last',x1Last,'x2Last',x2Last,'x3Last',x3Last);


%% agt structure
% Targets
tgt1_o_start    = [0;0;0];
tgt1_v          = vTGT;
tgt1_color      = 'r';
tgt1_marker     = '.';
tgt1_trace      = '--';
TGT1 = target(N,tgt1_o_start,tgt1_v,tgt1_color,tgt1_marker,tgt1_trace);

if pbm.nTGT==2,
    tgt2_o_start    = [0;-15;0];
    tgt2_v          = vTGT;
    tgt2_color      = 'm';
    tgt2_marker     = '.';
    tgt2_trace      = '--';
    TGT2 = target(N,tgt2_o_start,tgt2_v,tgt2_color,tgt2_marker,tgt2_trace);
end

% UAVs
uav1_s_start    = [0;30;zUAV];
uav1_psi_start  = 0;
uav1_v          = vUAV;
uav1_D          = [0;30;0];
uav1_color      = 'k';
uav1_marker     = 'x';
uav1_trace      = '-.';
UAV1 = uav(N,uav1_s_start,uav1_psi_start,uav1_color,uav1_marker,uav1_trace,uav1_v,uav1_D);

if pbm.nUAV==2,
    uav2_s_start    = [0;-30;zUAV];
    uav2_psi_start  = 0;
    uav2_v          = vUAV;
    uav2_D          = [0;-30;0];
    uav2_color      = 'b';
    uav2_marker     = 'o';
    uav2_trace      = '-.';
    UAV2 = uav(N,uav2_s_start,uav2_psi_start,uav2_color,uav2_marker,uav2_trace,uav2_v,uav2_D);
end

if      pbm.scn==1, agt=struct('UAV1',UAV1,'TGT1',TGT1);
elseif  pbm.scn==2, agt=struct('UAV1',UAV1,'UAV2',UAV2,'TGT1',TGT1);
elseif  pbm.scn==3, agt=struct('UAV1',UAV1,'UAV2',UAV2,'TGT1',TGT1,'TGT2',TGT2);
end


%% Clean workspace
clearvars -except pbm cst data agt gridX gridL gridV

