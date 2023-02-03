function [speed, steplength,stepfreq] = gaitspeed(w, varargin)
% [speed, steplength,stepfreq] = gaitspeed(w) returns the speed, steplength,
% and step frequency in dimensionless units for a gait w.
% w is a rimless wheel in 2-d, walkrw2
% optional: gaitspeed(w, xe, te) uses the end state and time and
% avoids doing another simulation

xs = get(w, 'xstar'); 
alpha = get(w,'alpha');

if nargin >= 5 % we are fed a step already, no need for simulation
  xe = varargin{1};  % end state
  te = varargin{2};
  xs = varargin{3};  % all states
  ts = varargin{4};
  if nargin == 7
    indices = varargin{6}; % indices
  else
  % use negative zero-crossings to estimate the number of steps
    zerocrossings = find((xs(1:end-1,1)>=0).*xs(2:end,1) < 0);
    indices = zerocrossings;
  end % if indices included
else % need to do a simulation
  [xe,te,x,t,energies,indices] = onestep(w);
end

steplength = 2*sin(alpha); % this is the distance traveled  
speed = length(indices)*steplength / te;
stepfreq = length(indices)/te;

