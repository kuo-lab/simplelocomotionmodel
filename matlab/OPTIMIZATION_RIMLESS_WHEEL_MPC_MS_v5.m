
%% Set up the path
clear; close all;
clc; beep off; 
currentFolder = pwd
addpath(genpath(currentFolder));
%%
speed_str = 'HallwayExperimentApprox1_50';%'alpha3_HallwayExperimentApprox1_0'%'HallwayExperimentApprox1_50'
load(strcat('w_',speed_str),'w'); 
w0 = w; clear w; alpha = get(w0,'alpha');
[stepLenNomRW, stepTimeNomRW, nominalP_RW, nominalLegSpeedRW, nominalPushOffWorkRW, nominalStepSpeedRW, nominalMiddleStanceTimeRW, nominalHeelStrikeWorkRW,...
    nominalHipTorqeWithoutPushOff, xsNomWithMS, tsNomWithMS, vintermsNom] = loadNominalsRW(w0, speed_str);

xInd = find(xsNomWithMS(:,1)==0) ;
nominalLegMSSpeedRW = xsNomWithMS(xInd,2);
x_init = xsNomWithMS(xInd, :);

variableStepLenFlag = 0;
constantSpeedFlag = 0;
constantTimeFlag = 0;
minimizeHorizontalPushfOffFlag = 0;

% select the terrain: oU, oD, UnD, oD2FFsBump (refers to DnUP), pyramid, longUnevenA (refers to complex 1), longUnevenB (refers to complex 2)
% if you select oU_no_opt it walks over oU without optimizing, 
% level_no_opt01  flat terrain no optimization
trialTypeTempStr = 'longUnevenB';%'UnD';%'level_no_opt01';%';%'traj';%''oD2FFsBump';%'pyramid';%'longUnevenA';%';% _variableStepLenOLDver  'oD';%'oU';%UnD_variableStepLen' 'level_no_opt16';%'pyramid';%'oD2FFsBump';;%'oU'%';%'oU';%'longUnevenA';

if strfind(trialTypeTempStr, 'variableStepLen')
    variableStepLenFlag = 1; 
    nominalSpeedForVariableStep = abs(nominalLegMSSpeedRW);
end

runNonLinModel = 0;
latePushOffFlag = 0;
includeHipTorqueFlag = 0;

%figure
FZ = 15; LW = 1;
colors = jet(length(1 ));% colors = jet(length(bumps_heights )); %colors = [1 0 0; 0 0 1; 0 0 0;  1 0 1; 0 1 1; 0 1 1];% r b k m c c
color_index = 1; color_index_diff = 1;
%********************************************************************************

options = optimset('algorithm', 'sqp', 'MaxFunEvals', 20000, 'Display', 'iter-detailed', 'TolX', 1e-8);% 50000,'TolFun',1e-8,'TolCon',1e-7); %'iter-detailed'

% i = 1; %no loop now
numStepsBefore = 6; % 6 ORIGIAL Numsteps = numStepsAfter +  numStepsBefore  + bumps. % MS when numStepsAfter =  numStepsBefore it's symmetric for oU and oD.
numStepsAfter =  numStepsBefore;

stdVal = [], offsetBump = [], uniformScale = []; bumpsHeightTraj = [];

%if running a traj
% bumHeightDimless = [0.125 -0.125];%0.075 is 3 inched 0.125 is 5 inches this is like 7.5cm
% bumpsHeightTraj = asin(bumHeightDimless/(2*sin(alpha)));

[bumps, numSteps, optimationOnFlag, constantTimeFlag ] = setBumps2v3(numStepsBefore, numStepsAfter, trialTypeTempStr, alpha, stdVal, offsetBump, uniformScale, bumpsHeightTraj);
boundPushOffBegin = []; boundPushOffEnd = [];
[params, lb, ub] = setInitialParamsNBoundsForRW_v2(numSteps, nominalP_RW, runNonLinModel, stepTimeNomRW, boundPushOffBegin, boundPushOffEnd)

w = set(w0,'bumps', bumps);         
tsTemp2 = []; xsTemp2 = []; %this are required when tere is no optimization
if optimationOnFlag          
    % cost 
    if variableStepLenFlag
        fcost = @(params)f_minimizeOptimalCostRW2_MStoMSvariableAlpha(w, params, latePushOffFlag, x_init, nominalSpeedForVariableStep);
    elseif constantTimeFlag
        fcost = @(params)f_minimizeOptimalCostRW2_MStoMS_constTime(w, params, x_init, stepTimeNomRW, latePushOffFlag);
    elseif minimizeHorizontalPushfOffFlag
        fcost = @(params)f_minimizeOptimalCostRW2_MStoMSminimizeHorizontalPushfOff(w, params, latePushOffFlag, x_init);
    else
        fcost = @(params)f_minimizeOptimalCostRW2_MStoMS(w, params); 
    end
   
    %constraint
    desiredTime = stepTimeNomRW*numSteps;
    
    if variableStepLenFlag
        fconstraint = @(params)gaitConstraintsForUnevenTerrainRW2_MStoMSvariableStepLen(params, w,  desiredTime, nominalLegMSSpeedRW, latePushOffFlag, x_init, nominalSpeedForVariableStep)
    elseif constantTimeFlag
        fconstraint = @(params)gaitConstraintsForUnevenTerrainRW2_MStoMS_constantTime(params, w, nominalLegMSSpeedRW, latePushOffFlag, x_init)
    elseif constantSpeedFlag
        fconstraint = @(params)gaitConstraintsForUnevenTerrainRW2_MStoMS_constantSpeed(params, w, nominalLegMSSpeedRW, latePushOffFlag, x_init)  
    else
        fconstraint = @(params)gaitConstraintsForUnevenTerrainRW2_MStoMS(params, w,  desiredTime, nominalLegMSSpeedRW, latePushOffFlag, x_init);
    end
    
    % optimize
    [optctrls, fval, exitflag, output, lambda, grad, hessian] = fmincon(fcost, params, [], [], [], [], lb, ub, fconstraint,options); 
    
    if exitflag ~= 1
        warning('Fmincon can not optimize \n');
        keyboard;
    end   

    %stop if the optimal solution hits to the upper bound this is %not reaseonable to do if there are limited PO's also just %check for upper bound PO can be zero
    optctrlsTocheck = optctrls(find(optctrls ~= nominalP_RW));%<--------ASSUMES first row is push ofss
    if ~isempty(find(optctrlsTocheck == max(ub(1,:)))) || ~isempty(find(optctrls == min(lb))) %<--------ASSUMES first row is push ofss
       error('hiting the bounds');
    end

    % regenerate    INCLUDE non line. model later      
    if variableStepLenFlag
        [xc, tc, xs, ts, energies, indices, vinterms] = onestep3_MStoMSvariableAlpha(w, x_init, [], optctrls, latePushOffFlag, nominalSpeedForVariableStep);  %Original 
    else
        [xc, tc, xs ,ts, energies, indices, vinterms] = onestep3_MStoMS(w, x_init, [], optctrls, latePushOffFlag);   
    end
             
else %NO OPTIMIZATION       
      optctrls = ones(1,numSteps(1))*nominalP_RW;
      [xc,tc,xs,ts,energies,indices, vinterms] = onestep3_MStoMS(w, x_init,[], optctrls, latePushOffFlag); 
end

outputORG = getSpeedWorkTimeGainRw2(xs, ts, indices, energies, optctrls, bumps, nominalPushOffWorkRW, stepLenNomRW, stepTimeNomRW);

figSpeed = figure; hold on;
plot(outputORG.timeMS_wrtHS, outputORG.speedMS , 'color', colors(1,:),'linewidth', LW);  plot(outputORG.timeMS_wrtHS, outputORG.speedMS,'.', 'markersize', 15, 'color', colors(1,:));
plot([0 0], ylim,'k-.','linewidth', LW); plot(xlim, -[nominalLegMSSpeedRW nominalLegMSSpeedRW], 'k--','linewidth', LW);   xlabel('time mid stance wrt on bump heel strike(dim.less)'); ylabel('angular vel mid stance');title(trialTypeTempStr);

figTimeGain = figure;
fms = plot(outputORG.timeMS, outputORG.timeGainMS, 'color', colors(1,:),'linewidth', LW); hold on; plot(outputORG.timeMS, outputORG.timeGainMS,'.', 'markersize', 15, 'color', colors(1,:));
plot(xlim, [0 0], 'k--','linewidth', LW); xlabel('time mid stance (dim.less)'); ylabel('time gain mid stance'); title(trialTypeTempStr);
fhs = plot(outputORG.timeMS, outputORG.timeGainHS,'r','linewidth', LW); plot(outputORG.timeMS, outputORG.timeGainHS,'r.','linewidth', LW, 'markersize', 15);  %you took out the inital optimal step
plot([0 0], ylim,'k-.','linewidth', LW); legend([fms fhs], 'Mid Stance time gain', 'Heel strike time gain')


figSpeedImu = figure;
plot(outputORG.timeNormalized, -xs(:, 2) , 'color', colors(1,:),'linewidth', LW); hold on; %
plot(outputORG.timeMS, outputORG.speedImu , 'color', colors(1,:),'linewidth', LW); hold on; plot(outputORG.timeMS, outputORG.speedImu,'.', 'markersize', 15, 'color', colors(1,:));
plot([0 0], ylim,'k-.','linewidth', LW); plot(xlim, [nominalStepSpeedRW nominalStepSpeedRW], 'k--','linewidth', LW);  ylim([0 0.8]); xlabel('time mid stance (dim.less)'); ylabel('imu speed'); title(trialTypeTempStr);

