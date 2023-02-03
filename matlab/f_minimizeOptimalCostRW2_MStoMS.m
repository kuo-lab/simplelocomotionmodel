function cost = f_minimizeOptimalCostRW2_MStoMS(w, params)

%cost function of rimless wheel to minimize total push off work

% inputs: w; walk object
%         params; push off trajectory (the decision variable)

% output: the cost total push off work. The push off work per step is 1/2 P^2 where P is the push off impulse

%Osman Darici

cost = 0.5*sum(params.^2);  %ORIGINAL

