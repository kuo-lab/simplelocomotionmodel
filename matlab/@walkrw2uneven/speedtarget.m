function [c,ceq] = speedtarget(x, w, speedtarget)

[xc,tc] = onestep(w,[],[], x(:)'); % apply controls here

%c = [max(xs(:,2))+0.37];  % here's how to apply a constraint to minimum
%speed
c = []; % no inequality constraints
ceq = xc(2)-speedtarget; 

end

