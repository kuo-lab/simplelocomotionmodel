function [] = plotstep(walk, x, t)
% PLOTSTEP   Produces a plot of interesting state information for walkrw2 
% [] = plotstep(t, x, walk) produces a time series plot of state information, given input
% consisting of time t, state x, and a walk object.

% CHANGES
%   Modified by Art 6/2009 to allow "hold on" on a figure, so that 
%   multiple onesteps can be superimposed.

if nargin == 0
  error('plotstep: need a walk object as first argument');
elseif nargin == 1 % optional arguments
  if ~isa(walk, 'walksw2')
        error('plotstep: need a walk object as first argument');
  end
  [xe,te,x,t] = onestep(walk); % run a simulation if no states given
elseif nargin == 2 % either given an initial state or a list of states
  if length(x) == 4   % and it's a vector, meaning an initial condition
    t = 0;
  elseif size(x,1) > 1 && size(x,2) == 4 % it's a matrix of states
    t = linspace(0,100,size(x,1)); % Time as a percentage of the length of the series of states
  else
    error('plotstep: unknown second argument')
  end
elseif nargin > 3
  error('incorrect number of arguments');
end

held = 'on';
if ~ishold, clf; held = 'off'; end
plot(t, x);
xlabel('Time'); ylabel('States');
