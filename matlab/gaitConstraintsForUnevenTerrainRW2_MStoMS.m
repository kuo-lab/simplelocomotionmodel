function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS(params, w,  desiredTime, nominalLegSpeedRW, latePushOffFlag, x_init)

% function [c,ceq] = gaitConstraintsForUnevenTerrainRW2_MStoMS(params, w,  stepTimeNomRW, nominalStepSpeedRW, nominalLegSpeedRW, numSteps, latePushOffFlag,...
%     numStepsBefore, optimizationWithConstantPushOffFlag, constantSpeedFlag, x_init, te_beginOnBump, timeSettingParam, desiredTime, constStr, epsSpeed, epsTime, limitNumPOtoEnd)


[xe, te, xs, ts, ~, indices, ~] = onestep3_MStoMS(w, x_init, [], params, latePushOffFlag);%Original 

% [xe, te, xs, ts, ~, indices, ~] = onestep3_MStoMS_MPC(w, x_init, [],
% params, latePushOffFlag);% this and above line for now generate the same thing and this function was used within
% the MPC constraints but not anymore. You created this function for MPC to do some MPC related stuff but they are not valid not so they do the same 26 DEC 2021


ceq = [xe(2)-nominalLegSpeedRW; te-desiredTime];
c = [];


% if ~constantSpeedFlag
%     if numStepsBefore == 1  && strcmp(constStr, 'original')
%         ceq = [xe(2)-nominalLegSpeedRW; te-(stepTimeNomRW*numSteps + stepTimeNomRW-te_beginOnBump)*timeSettingParam ]; %currently working
%          % stepTimeNomRW-te_beginOnBump to make the model and the twin brother begin both their feet is on bump, this is good for oU and UnD 
%          c = [];
%     else  
%         if strcmp(constStr, 'original')
%             ceq = [xe(2)-nominalLegSpeedRW; te-stepTimeNomRW*numSteps*timeSettingParam];  %ORIGINAL for paper 
% %             ceq = [xe(2)-nominalLegSpeedRW];  %trying fo elife
% %             ceq = [te-stepTimeNomRW*numSteps*timeSettingParam];
% %             ceq = [];
%             c = [];
%         elseif strcmp(constStr, 'timeGain')
% %             ceq = [xe(2)-nominalLegSpeedRW; (stepLenNom*numSteps)/te-desiredTime];  %desiredTime is used as desiredSpeedthis must be level walking by definiton
%             ceq = [xe(2)-nominalLegSpeedRW; te-(stepTimeNomRW*numSteps-desiredTime)];
%             c = [];
%         elseif strcmp(constStr, 'speedPercent')
%             ceq = te-stepTimeNomRW*numSteps*timeSettingParam;  % 
%             c = abs(xe(2)-nominalLegSpeedRW)-abs(nominalLegSpeedRW)*epsSpeed/100; %<-----------epsSpeed2 
%             
%         elseif strcmp(constStr, 'speedNumPOtoEndPercent')
%             ceq = [te-stepTimeNomRW*numSteps*timeSettingParam; xs(indices(limitNumPOtoEnd),2)-nominalLegSpeedRW];  %
%             c = abs(xe(2)-nominalLegSpeedRW)-abs(nominalLegSpeedRW)*epsSpeed/100; %<-----------epsSpeed2     
%             
%         elseif strcmp(constStr, 'speedNumPOtoEndPercentOrg')
%             ceq = [xe(2)-nominalLegSpeedRW; te-stepTimeNomRW*numSteps*timeSettingParam; xs(indices(limitNumPOtoEnd),2)-nominalLegSpeedRW];  %
%             c = [];
%         elseif strcmp(constStr, 'speedTimePercent')
% %             ceq = [];  
% %             c = [abs(xe(2)-nominalLegSpeedRW)-abs(nominalLegSpeedRW)*epsSpeed/100 ...
% %                  abs(te-stepTimeNomRW*numSteps) - stepTimeNomRW*numSteps*epsTime/100]; 
% 
%             ceq = xe(2)-nominalLegSpeedRW;  
%             c = [abs(te-stepTimeNomRW*numSteps) - stepTimeNomRW*numSteps*epsTime/100];
% 
%         elseif strcmp(constStr, 'timeGainspeed1Percent') 
%             ceq = te - (stepTimeNomRW*numSteps-desiredTime);%  %this must be level walking by definiton\
% %             keyboard();%change the condition name
%             c = abs(xe(2)-nominalLegSpeedRW)-abs(nominalLegSpeedRW)*epsSpeed/100; 
%         else
%             error('constraint error');
%         end
%     end
% else   
%     %old***************
%     alpha = get(w,'alpha');
% %     stepLen =    sin(abs(xs(indices,1))) +  sin(2*alpha-abs(xs(indices,1)));
% %     stepTime = [ts(indices(1),1);  diff(ts(indices,1))];         
% %     %          comDist =     [comDist     sin(abs(xs(indices(z),1))) +  sin(   2*alpha - abs(xs(indices(z-1),1))       )];                
% %     speed = stepLen./stepTime; %comDist./stepTime;
% %     ceq = [ speed - nominalStepSpeedRW];%te-stepTimeNomRW*numSteps; xe(2)-nominalLegSpeedRW; 
%      %old***************
%     
%      %for ELIFE constant speed
%      %makes the mid stance vel. equal to nom. mid stance vel. at each step
%      stepLenNomRW = nominalStepSpeedRW*stepTimeNomRW;
%      nominalMiddleStanceTimeRW = 0.832614444090635;
%      
%     nominalMSspeed = -0.439981524308518;
%     
%     [speed, xsNew, tsNew, middleStanceIndex, indicesNew] = getMiddleStanceSpeed_exponential(xs,ts,indices, alpha,  numStepsBefore,...
%     stepLenNomRW, stepTimeNomRW, nominalMiddleStanceTimeRW, te_beginOnBump);
%      ceq = [xsNew(middleStanceIndex,2)-nominalMSspeed; ];%te-stepTimeNomRW*numSteps
%      c = [];
%     
%     
% end






 
 