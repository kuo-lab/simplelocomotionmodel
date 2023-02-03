function w = walkrw2(varargin)
% WALKRW2 Create a 2-d rimless wheel walking gait
%   w = walkrw2 creates a default rw2 gait
%   w = walkrw2('normal') creates a normal gait with speed 1.25 m/s
%   w = walkrw2('slow') creates a slow rw2 gait, speed 0.1, Kp = 0
% Parent is walksw2, simplest walking model

switch nargin
case 0
  % if no arguments, create a default gait
  
% A default gait that gives close to equivalent of speed 1.25 m/s
   wsw2 = set(walksw2('normal'), 'P', 0.1475, 'Kp', 0, 'xstar', [ 0.3 -0.476827 ], 'N', 2);
   w.parms.rgyr = 0; w.parms.alpha = 0.3;
   w.parent = 'walksw2';
   w = class(w, 'walkrw2', wsw2);  
%   [w,cnvrg] = gradsearch(w, [], 'info', 0);
case 1
  if isa(varargin{1}, 'walksw2')
    wsw2 = varargin{1}; 
    w.parms.rgyr = 0; w.parms.alpha = 0.3;
    w.parent = 'walksw2';
    w = class(w, 'walkrw2', wsw2);
  elseif isempty(varargin{1}) % we want an empty object
    % empty objects are used when another object inherits methods from
    % the walksw2 class
    w.parms.g = []; w.parms.M = []; w.parms.L = []; 
    w.parms.R = []; 
    w.parent = 'walksw2'; wsw2 = walksw2([]);
    w = class(w, 'walk2', wsw2);
  elseif strcmp(varargin{1}, 'slow')
    % Here is a gait that gives equivalent of speed 0.1, with
    % no hip spring
    % P = 0.0133, Kp = 0, xstar = [ 0.145228 -0.145228 -0.158617 -0.15151 ]
    fname = which('walk2/slowgait.mat'); loadvar = load(fname);
    w = loadvar.w; 
  elseif strcmp(varargin{1},'normal')
    fname = which('walk2/normalgait.mat'); loadvar = load(fname);
    w = loadvar.w;
  end
end
