function [value,isterminal,direction] = heelstrikeevent(t,x,walk,parms)
% HEELSTRIKEEVENT Returns event location info for detecting heelstrike,
% where we'll stop the simulation.
% [value, isterminal, direction] = heelstrikeevent(t, x, walk)
% where t is time, x is the state, and parms is a walk is a walk object containing
% the parameters

% Arthur D. Kuo, see:
% Kuo, A. D. (2002) Energetics of actively powered locomotion using the 
%   simplest walking model, Journal of Biomechanical Engineering, 124: 113-120. 
% Kuo, A. D. (2001) A simple model predicts the step length-speed 
%   relationship in human walking, Journal of Biomechanical Engineering, 
%   123: 264-269. 

% CHANGES
%   Modified by Art to remove the ugly kludge by which we biased things for
%   long-period gaits, and avoidance of mid-stance scuffing.

% we don't need any parms, so this is currently unused
%if nargin < 4 % we are not given parameters
  %parms = get(walk, 'parms');
%end
  
x0 = get(walk, 'xstar');

% DEPRECATED:
%q10 = (x0(1) + parms.gamma); % this is an ugly kludge to avoid stumbling
% if the swing foot scuffs the ground. It is safer in future to make the
% modification to isterminal, and leave value alone.

q1 = x(1); q2 = x(2); u1 = x(3); u2 = x(4);

% DEPRECATED:
% look for a combination of the foot hitting the ground and
% being more than 10% of the original initial stance angle
% (to ignore stubbing of the foot near midstance) and
% also u2 < 0 (to look for long-period gaits)
% value = (-q2 + 2*q1) * (q1 <= -0.1*q10) * (u2 < 0);
value = (-q2 + 2*q1); 
isterminal = q1 < 0;   % stop if we've gone past midstance
direction = -1;   % and going negative
