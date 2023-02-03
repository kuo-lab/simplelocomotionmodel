function [speed, steplength,stepfreq] = gaitspeed(w)
% [speed, steplength,stepfreq] = gaitspeed(w) returns the speed, steplength,
% and step frequency in dimensionless units for a gait w.
% w is a rimless wheel in 2-d, walkrw2
xs = get(w, 'xstar'); 
[xe,te,x,t] = onestep(w);
steplength = 2*sin(xs(1)); 
speed = steplength / te;
stepfreq = 1/te;

