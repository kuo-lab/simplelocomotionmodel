function w = walkrw2unevenl(varargin)
% WALKrw2uneven Create a rimless wheel 2-d walking gait for uneven terrain,
% linearized dynamics
%   w = walk2 creates a default sw2 gait
%   w = walk2('normal') creates a normal gait with speed 1.25 m/s, 1.8 Hz
%   w = walk2('slow') creates a slow sw2 gait, speed 0.1, Kp = 0
switch nargin
case 0
  % if no arguments, create a default gait
  
% A default gait that gives close to equivalent of speed 1.25 m/s, 1.8 Hz
  wrw2u = walkrw2uneven;
  w.parent = 'walkrw2uneven';
  w = class(w, 'walkrw2unevenl', wrw2u);  

  case 1
  if isa(varargin{1}, 'walkrw2uneven')
    wsw2u = varargin{1}; 
    w.parent = 'walkrw2uneven';
    w = class(w, 'walkrw2unevenl', wsw2u);
  elseif isempty(varargin{1}) % we want an empty object
    % empty objects are used when another object inherits methods from
    % the walksw2 class
    w.parms.g = []; w.parms.Mp = []; w.parms.M = []; w.parms.L = []; w.parms.C = [];
    w.parms.R = []; w.parms.Ip = []; w.parms.Il = [];
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
