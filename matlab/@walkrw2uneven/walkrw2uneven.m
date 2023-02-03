function w = walkrw2uneven(varargin)
% WALKrw2uneven Create a rimless wheel 2-d walking gait for uneven terrain
%   w = walk2 creates a default sw2 gait
%   w = walk2('normal') creates a normal gait with speed 1.25 m/s, 1.8 Hz
%   w = walk2('slow') creates a slow sw2 gait, speed 0.1, Kp = 0
switch nargin
case 0
  % if no arguments, create a default gait
  
% A default gait that gives close to equivalent of speed 1.25 m/s, 1.8 Hz
  wrw2 = walkrw2;
  w.parent = 'walkrw2';
  w.parms.bumps = 0; w.parms.controls = NaN; % bumps is a list of angle changes, controls is array of push-off/hip adjustments
  w.parms.latePushOffFlag  = 0; %<----------------------------
  w.parms.T  = 0; %<----------------------------
  w = class(w, 'walkrw2uneven', wrw2);  

  case 1
  if isa(varargin{1}, 'walksw2')
    wsw2 = varargin{1}; 
    w.parms.g = 1;
    w.parms.Mp = 0.68; w.parms.M = 0.16; w.parms.L = 1; w.parms.C = 0.645; w.parms.R = 0.3;
    w.parms.Ip = 0; w.parms.Il = w.parms.M* (0.326*w.parms.L)^2;
    w.bumps = 0; w.controls = 0; % bumps is a list of angle changes, controls is array of push-off/hip adjustments
    w.parent = 'walkrw2';
    w = class(w, 'walk2', wsw2);
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
