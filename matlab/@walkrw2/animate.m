function animate(w,varargin)
% animate(w) 
%   animates the 2-d rimless wheel by w.
% animate(w, 'numsteps', N) automatically repeats the simulation N times (default 2).
% animate(w, x, stepindices) animates w with the given states given in x
%   (rows of the state vector), and with stepindices containing a list
%   of the number of frames in each step.
% animate(w, 'arrows', 1) also draws arrows for COM velocity
% animate(w, 'save', 1) animates w and saves each frame as an 
%   adobe illustrator file
% animate(w, 'frames', Nframes) draws a certain number of frames per step

% Art Kuo

parms = get(w,'parms'); alpha = parms.alpha;

%footn = 10; % how many segments to draw in the curved foot

debg = 0; % set to 1 to help debugging

numsteps = 2; saveflag = 0; arrowflag = 0; nframes = 16; % default values
x0 = []; stepindices = [];

if nargin == 0
  error('animate: need a walk object as argument');
elseif nargin > 1 % optional arguments
  % figure out if the second argument is an initial condition vector, or a
  % bunch of rows of states
  property_argin = varargin;
  secondargument = property_argin{1};
  if isa(varargin{1}, 'double')      % second argument appears to be a number
    if length(secondargument) == 4   % and it's a vector, meaning an initial condition
      x0 = secondargument;
      property_argin = property_argin(2:end); 
    elseif size(secondargument,1) > 1 && size(secondargument,2) == 2 % it's a matrix of states
      xs = varargin{1}; stepindices = varargin{2}; numsteps = length(stepindices);
      property_argin = property_argin(3:end);
    else
      error('animate: unknown second argument')
    end
  end
  % Step through the optional arguments
  while length(property_argin) >= 2,
    prop = property_argin{1};
    val = property_argin{2};
    property_argin = property_argin(3:end);
    switch prop
      case 'numsteps'
        numsteps = val;
      case 'save'
        saveflag = val;
      case 'arrows'
        arrowflag = val;
      case 'frames'
        nframes = val;
    end
  end
end

if isempty(stepindices) % we've haven't been supplied with a bunch of states
  [xe,te,xs,ts] = onestep(w, x0, 'anim', nframes); stepindices = length(xs);
end

xlen = length(xs);

% if length(stepindices) == 1 % there's just one step stored in xs
%   stepindices = [xlen repmat(xlen-1, 1, numsteps-1)];
%   startindex = repmat(1,numsteps,1); % extra frame
%   endindex = repmat(xlen, numsteps, 1);
% elseif length(stepindices) > 1  % there's more than one step in xs
%   endindex = cumsum(stepindices);
%   startindex = [1 endindex-1];
% end
% Now numsteps contains the number of steps, stepindices
% contains the number of frames in each step, and
% startindex and endindex contain indices for each step

% Estimate range of walking
distance = (numsteps+1)*2*sin(alpha);
xlimit = [0 distance]-2*sin(alpha); 
ylimit = [-0.05 2.05];

% arrow parameters
aang = pi/6; scale = 0.02; scale2 = 2; vx2 = 0.4; vy2 = 1.2;




% Initialize
clf; axis equal
%hold on



h = drawmodel(w, xs(1,:), [0;0]);
for i = 1:xlen
  x = xs(i,:);
  drawmodel(w, x, [0; 0], h);
  drawnow; pause(0.05);
end

