function [value,isterminal,direction] = heelstrikeevent(t,x,walk,parms)
% HEELSTRIKEEVENT Returns event location info for detecting heelstrike,
% where we'll stop the simulation.
% [value, isterminal, direction] = heelstrikeevent(t, x, walk)
% where t is time, x is the state, and parms is a walk is a walk object containing
% the parameters
% This is for the 2-D rimless wheel

% McGeer (1990) Passive dynamic walking, Intl. J. Robotics Research,
%   9: 62-82.

if nargin < 4 || isempty(parms)
  parms = get(walk, 'parms');
end

alpha = parms.alpha;

q1 = x(1); u1 = x(2); 

value = q1 + alpha; % q1 is measured ccw from vertical
% and wheel rolls to the right (cw), so q1 is negative
% at heel strike

isterminal = 1;   % always stop
direction = -1;   % and going negative
