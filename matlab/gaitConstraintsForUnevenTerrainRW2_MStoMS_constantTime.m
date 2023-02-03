function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS_constantTime(params, w, nominalLegSpeedRW, latePushOffFlag, x_init)

% function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS(params, w,  stepTimeNomRW, nominalStepSpeedRW, nominalLegSpeedRW, numSteps, latePushOffFlag,...
%     numStepsBefore, optimizationWithConstantPushOffFlag, constantSpeedFlag, x_init, te_beginOnBump, timeSettingParam, desiredTime, constStr, epsSpeed, epsTime, limitNumPOtoEnd)


[xe, te, xs, ts, ~, indices, ~] = onestep3_MStoMS(w, x_init, [], params, latePushOffFlag);%Original 


ceq = [xe(2)-nominalLegSpeedRW; ];
% ceq = [];
c = [];


