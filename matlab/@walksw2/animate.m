function animate(w,varargin)
% animate(w) 
%   animates the 2-d passive walking model described by w.
% animate(w, 'numsteps', N) automatically repeats the simulation N times (default 2).
% animate(w, x, framecounts) animates w with the given states given in x
%   (rows of the state vector), and with framecounts containing a list
%   of the number of frames in each step, e.g. [16 16 16] for 3 steps.
% animate(w, 'arrows', 1) also draws arrows for COM velocity
% animate(w, 'save', 1) animates w and saves each frame as an 
%   adobe illustrator file
% animate(w, 'frames', Nframes) draws a certain number of frames per step
% Other options: 'treadmill' display moving treadmill or walk overground;
%   'framedelay' (0.05) how long to wait between frames of animation;
%   'groundlinewidth' how thick the ground line should be, in points;
%   Also drawmodel options such as 'modellinewidth', 'showcomvel', and
%   'showgrfs' (show arrows for COM velocity or ground reaction forces)      

% Art Kuo
% Dependency: This calls drawmodel

% allow a keypress or button press in the figure to stop the animation
keypressed = 0;
set(gcf,'KeyPressFcn', @keypress, 'ButtonDownFcn', @keypress); 

treadmill = 1; % by default, draw a treadmill

%parms = get(w,'parms'); % access model parameters
N = get(w, 'N'); L = 1; R = 0;

% dimensions of objects to be drawn
floorheight = 0.02; 
modellinewidth = 4; groundlinewidth = 2; jointsize = 10; footsize = 6;
footn = 10; % how many segments to draw in the curved foot

debg = 0; % set to 1 if you want a long pause between frames

numsteps = 2; saveflag = 0; nframes = 16; % default values
framedelay = 0.05;

showcomvel = 0; showgrfs = 0;

x0 = []; framecounts = []; % default is to use fixed point

% Allow for flexible input arguments, particularly allowing second argument
% to be x, a list of state vectors to animate. Otherwise assume the
% inputs are property/value pairs.
if nargin == 0
  error('animate: need a walk object as argument');
elseif nargin > 1 % optional arguments
  % figure out if the second argument is an initial condition vector, or a
  % bunch of rows of states
  property_argin = varargin;
  secondargument = property_argin{1};
  if isa(varargin{1}, 'double')      % second argument appears to be a number
    if length(secondargument) == N   % and it's a vector, meaning an initial condition
      x0 = secondargument;
      property_argin = property_argin(2:end); 
    elseif size(secondargument,1) > 1 && size(secondargument,2) == N % it's a matrix of states
      xs = varargin{2}; framecounts = varargin{3}; numsteps = length(framecounts);
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
        saveflag = val; if saveflag, numsteps = 1; end; % only save one step by default
      case 'frames'
        nframes = val;
      case 'treadmill'
        treadmill = val;
      case 'showgrfs'
        showgrfs = val;
      case 'showcomvel'
        showcomvel = val;
      case 'framedelay'
        framedelay = val;
      case 'modellinewidth'
        modellinewidth = val;
      case 'groundlinewidth'
        groundlinewidth = val;

    end
  end
end

% send this to drawmodel
drawmodeloptions = {'modellinewidth', modellinewidth, 'footn', footn, ...
  'jointsize', jointsize, 'footsize', footsize, 'showcomvel', showcomvel, 'showgrfs', showgrfs};

if isempty(framecounts) % we've haven't been supplied with a bunch of states
  [xe,te,xs,ts] = onestep(w, x0, 'anim', nframes); framecounts = length(xs);
end

speedinfo = walkspeed(w); sl = speedinfo.steplength;

% Now numsteps contains the number of steps, framecounts
% contains the number of frames in each step, and
% startindex and endindex contain indices for each step

% Estimate range of walking
distance = numsteps*sl; 

if treadmill % make room for treadmill or overground steps
  xlimit = [-(sl+R)*1.1 sl*1.2];    % horizontal
else
  xlimit = [-(sl+R) distance+0.5*R];% 
end    
ylimit = [-floorheight-0.04 1.02*L];        % vertical limit
if showgrfs, ylimit(1) = ylimit(1) - 0.1; end; % extra room for grf arrows

% Initialize figure with equal axes with correct limits
clf; set(gcf, 'color', [1 1 1]); 
set(gca,'DataAspectRatio',[1 1 1],'Visible','off','NextPlot','Add','XLim',xlimit,'YLim',ylimit);
set(gca,'Units', 'Points'); 
axesinpoints = get(gca,'Position'); ptstodata = diff(xlimit)/axesinpoints(3);

if saveflag % need to provide a bounding box to make the frames consistent when opened in Flash
  hboundingbox = rectangle('Position',[xlimit(1) ylimit(1) diff(xlimit) diff(ylimit)],'Visible','On','FaceColor','w','EdgeColor','none');
end

% Drawmodel will plot the foot's ground contact at (0,0) so let's plot
% the ground and tiles underneath that, taking into account the thickness
% of the lines.
%ygndoffset = -0.5*(groundlinewidth+modellinewidth)*ptstodata; % for curved feet
ygndoffset = -0.5*(groundlinewidth+footsize)*ptstodata; % for point feet
ytileoffset = ygndoffset - 0.5*groundlinewidth*ptstodata;
hgnd = line(xlimit,[1 1]*ygndoffset,'color',[0 0 0],'linewidth',groundlinewidth);

% fill the floor with tiles extending horizontally two extra 
% tiles to the right (one black, one white) so that when the treadmill 
% scrolls left, we always show all tiles

tilelen = sl/4; % tile length, equal to an integer fraction of step length
% tilexs is x positions across the screen, while also giving an odd number 
% of vertices horizontally, for an even number of tiles. 

% Make an odd number of vertices to stretch across the floor, for an 
% even number of tiles.
Nc = floor(diff(xlimit)/tilelen) + 1; if mod(Nc,2)==0, Nc = Nc + 1; end;
tilexs = xlimit(1) + (0:Nc-1)*tilelen;
% Tiles are defined by vertex and face matrices
% where the vertices are rows of x-y-z triples, in two groups of rows.
% The first group is the vertices stretching across the screen at ground
% height, and the second group (below the first) is the same set of
% vertices but at floorheight below ground. 
%  1      2      3      4      5
%  *------*      *------*      *    (We end up not drawing the last
%  |-  I -|      |- II -|      |     vertices on the right since they are 
%  *------*      *------*      *     white) 
%  6      7      8      9     10     

tilevm = [tilexs' ones(Nc,1)*ytileoffset zeros(Nc,1);
  tilexs' ones(Nc,1)*ytileoffset-floorheight zeros(Nc,1)];
% define dark faces from the vertices (white faces are not drawn)
tilefm = repmat((1:2:Nc-1)',1,4) + repmat([0 1 Nc+1 Nc],floor(Nc/2),1);
hgndtiles = patch('Vertices', tilevm, 'Faces', tilefm);

% contactpoint with floor can move backwards if we're on
% a treadmill, or it stays in one place. For additional
% steps it has to move forward at each s2s transition.

startpoint = 0; % start us forward so it's approximately
% centered on the treadmill
if treadmill == 0, startpoint = -xs(1,1)+R; end % or try zero

h = drawmodel(w, xs(1,:), [startpoint;0], [], drawmodeloptions{:});

cntr = 1; % absolute frame counter

for j = 1:numsteps % Loop through each step
  if treadmill % we'll move the model with the treadmill
    modelorigin = startpoint; 
    dx = sl / framecounts(min(end,j)); % how much to move the treadmill
  else         % or allow it to advance down the floor
    modelorigin = startpoint + (j-1)*sl;
    dx = 0;
  end
  for i = 1:framecounts(min(end,j)) % each step has this many frames
    drawmodel(w, xs(i,:), [modelorigin; 0], h, drawmodeloptions{:});
    set(hgndtiles,'Vertices', tilevm); % update treadmill vertices
    
    modelorigin = modelorigin - dx; % move model to left    
    tilevm(:,1) = tilevm(:,1) - dx; % and move treadmill to the left
    
    if tilevm(2,1) < xlimit(1), % scrolled a whole tile to left
      tilevm(:,1) = tilevm(:,1) + 2*tilelen;  % reset two tiles to the right
    end
        
    drawnow; pause(framedelay)

    if saveflag
      print('-depsc2',sprintf('walk%02d',cntr));
      %print('-depsc2',sprintf('walk%02d',cntr));

      % Note Matlab is threatening to remove Illustrator format. The
      % alternative is to use -depsc2, which does not import directly
      % into Flash. We have to import the eps into Illustrator and then
      % export from Illustrator to Flash format. See below for more info.
    end
    
    if debg, pause, end;
    cntr = cntr + 1;
    if keypressed
      return
    end
    
  end % frame loop 
  
end % numsteps loop

  function keypress(src,eventdata)
    keypressed = 1;
  end
  
end

%
% 1. DILL AI -> Flash -> SWF
%    Export as Illustrator, then use Flash to read the Illustrator files,
%    using import to stage. You can select multiple files. Then publish
%    as SWF. Need to fix size in Flash, where editing is possible.
%
% 2. EPSC2 -> ILL CS5 -> AI -> Flash
%    Export as EPSC2, then use Illustrator to read EPS using script.
%    Then save as Illustrator. Use Flash to read AI file using
%    import to stage. This is editable in Flash and can be published
%    with SWF, but you have to set the stage size.
%
% 3. EPSC2 -> ILL CS5 -> SWF export
%    Export as EPSC2, then use Illustrator to read EPS using script.
%    Then export as SWF. Works fine except lines come out rounded 
%    ends. Very fast and usable. But cannot read the SWF as editable
%    file in Flash. 
%
% 4. EPSC2 -> ILL CS5 (embed) -> SWF export [bad method]
%    Export as EPSC2, then use Illustrator to read EPS using script.
%    If the linked files are converted to embed, the exported SWF has
%    screwed up lines. Not recommended. Of course the Illustrator file
%    can be saved as AI and read by Flash and used.


