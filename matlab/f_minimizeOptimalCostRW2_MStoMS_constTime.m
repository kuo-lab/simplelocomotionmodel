function cost = f_minimizeOptimalCostRW2_MStoMS_constTime(w, params, x_init, stepTimeNomRW, latePushOffFlag)


% cost function of rimless wheel to minimize the deviation of each step's
% duration from nominal walking step time

% inputs: w; walk object
%         params; push off trajectory (the decision variable)
%        x_init the initial state that the model begins to walk
%        stepTimeNomRW; nominal step time of the gait
%        latePushOffFlag the flag that determines whether the pus off is preemeptive (applied before collision happens) or not

% output: the total time deviation of each step's duration from nominal walking step time

%Osman Darici

[xe, te, xs, ts, ~, indices, ~] = onestep3_MStoMS(w, x_init, [], params, latePushOffFlag);

stepTime = [ts(indices(1)) diff(ts(indices))'];

%minimize the deviation of eac step's time from nominal step time
cost = sum((stepTime - stepTimeNomRW).^2);