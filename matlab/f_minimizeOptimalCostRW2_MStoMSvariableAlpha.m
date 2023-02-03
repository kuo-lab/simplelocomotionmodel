
function cost = f_minimizeOptimalCostRW2_MStoMSvariableAlpha(w, params, latePushOffFlag, x_init, nominalStepSpeedRW)

%cost = 0.5*sum(params.^2);  %ORIGINAL

%Variable alpha with swing cost
% [xe,te,x,t,energies,indices,vinterms, alphaNewAll] = onestep3variableAlpha(w, x0, bumps, ctrls, latePushOffFlag, nomMidStanceSpeed, varargin);

[xe,te,x,t,energies,indices,vinterms] = onestep3_MStoMSvariableAlpha(w, x_init, [], params, latePushOffFlag, nominalStepSpeedRW);
indicesHS = indices-10;
stepLen = sin(abs(x(indicesHS,1))) +  sin(abs(x(indicesHS+1,1)));
stepTime = [t(indices(1));  diff(t(indices))];
% cost = 0.5*sum(params.^2) +  0.0115  * sum((stepLen./stepTime).^0.42 .* (1./t(indices)).^3); %this is  C*(speed.^0.42)*freq^3;   The mominal coef is 0.0115 which is number found from SW2 simulations


costCoef = 0.05;%0.0115;%0.05;%0.15;%0.0115;
cost = 0.5*sum(params.^2) +  costCoef  * sum((stepLen./stepTime).^0.42 .* (1./stepTime).^3); %this is  C*(speed.^0.42)*freq^3;   The mominal coef is 0.0115 which is number found from SW2 simulations


% [~, ~, xs, ts, ~, indices, ~, ~] = onestep3variableAlpha(w, x_init, [], params2, latePushOffFlag, nominalMSspeed);   %this is to try for varibale alpha for eLife
% alpha = get(w, 'alpha');
% [comDistMstoMs, comTimeMstoMs ] = getComDistNTimeMstoMS_variableAlphaRW(xs,ts,indices, alpha, numStepBefore, stepLenNomRW, stepTimeNomRW, nominalMiddleStanceTimeRW, te_beginOnBump);
% 
% cost = 0.5*sum(params2(1,:).^2) +  0.0115  * sum((comDistMstoMs./comTimeMstoMs).^0.42 .* (1./comTimeMstoMs).^3); %this is  C*(speed.^0.42)*freq^3;   The mominal coef is 0.0115 which is number found from SW2 simulations


