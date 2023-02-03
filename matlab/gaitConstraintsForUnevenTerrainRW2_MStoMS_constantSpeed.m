function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS_constantSpeed(params, w, nominalMSspeed, latePushOffFlag, x_init)

% function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS(params, w,  stepTimeNomRW, nominalStepSpeedRW, nominalLegSpeedRW, numSteps, latePushOffFlag,...
%     numStepsBefore, optimizationWithConstantPushOffFlag, constantSpeedFlag, x_init, te_beginOnBump, timeSettingParam, desiredTime, constStr, epsSpeed, epsTime, limitNumPOtoEnd)


[xe, te, xs, ts, ~, indices, ~] = onestep3_MStoMS(w, x_init, [], params, latePushOffFlag);%Original 


ceq = ts(indices, 2)-nominalMSspeed;%constant speed
% ceq = ts(indices)-desiredTime;

c = [];


