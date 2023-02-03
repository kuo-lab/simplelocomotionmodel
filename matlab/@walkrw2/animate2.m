function animate2(w,varargin)
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
    elseif size(secondargument,1) > 1 %&& size(secondargument,2) == 4 % it's a matrix of states %<------------------
      xs = varargin{1}; stepindices = varargin{2}; numsteps = length(stepindices); %<------------------
%       bumps = varargin{3}; %<------------------
      property_argin = property_argin(3:end);
    else
      error('animate: unknown second argument')
    end
  end
  % Step through the optional arguments
  while length(property_argin) >= 3,%<------------------
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

LW = 0.5;
bumps = get(w,'bumps');
% figPos = [500, 500, 300, 100];%[100, 100, 1800, 1800];
% fig1 = figure('Position', figPos); 
fig1 = figure;
hold on; axis equal
h = drawmodel(w, xs(1,:), [0;0]); set(gcf,'Color','w'); axis off;
nom_steplen = 0.5910;
xlim([-2*nom_steplen  (length(bumps)+3)*nom_steplen]); xlimTemp = xlim;
% line(xlim, [0 0],'Color','Black', 'LineWidth',LW); % do not plot a whole line

%find and draw bump
bump_len = 1.1*nom_steplen;
bump_index = find(bumps ~= 0); bump = bumps(bump_index(1));
bump_mag = cos(alpha-bump) - cos(alpha+bump);
coef = 0.95;

l_left_corner = [bump_index(1)*nom_steplen*coef; 0];
line([xlimTemp(1) l_left_corner(1)], [0 0],'Color','Black', 'LineWidth',LW);
u_left_corner = [bump_index(1)*nom_steplen*coef; bump_mag];
u_right_corner = [bump_len+bump_index(1)*nom_steplen*coef; bump_mag];
l_right_corner = [bump_len+bump_index(1)*nom_steplen*coef; 0];


upOrDowFlag = 0;
if length(bump_index) == 1   
    upOrDowFlag = 1;
    h1 = line([l_left_corner(1) u_left_corner(1)], [l_left_corner(2) u_left_corner(2)], 'LineWidth',LW,'Color','Black');
    h2 = line([u_left_corner(1) u_right_corner(1)],[u_left_corner(2) u_right_corner(2)], 'LineWidth',LW,'Color','Black');
    h3 = line([l_right_corner(1) xlimTemp(2)], [u_left_corner(2) u_right_corner(2)],'Color','Black', 'LineWidth',LW);
else
    h1 = line([l_left_corner(1) u_left_corner(1)], [l_left_corner(2) u_left_corner(2)], 'LineWidth',LW,'Color','Black');
    h2 = line([u_left_corner(1) u_right_corner(1)],[u_left_corner(2) u_right_corner(2)], 'LineWidth',LW,'Color','Black');
    h3 = line([u_right_corner(1) l_right_corner(1)],[u_right_corner(2) l_right_corner(2)], 'LineWidth',LW,'Color','Black');
    h4 = line([l_right_corner(1) xlimTemp(2)], [0 0],'Color','Black', 'LineWidth',LW);
end
% area([l_left_corner(1) l_right_corner(1) ],[bump_mag bump_mag])

% h = drawmodel(w, xs(1,:), [0;0]);
shiftx = 0; shifty = 0;

vidObj = VideoWriter('vid_XX.avi');
% vidObj.FrameRate = 60;
open(vidObj);



nlegs = floor(pi / (2*alpha)); % roughly the right number of legs
hnlegs = floor(nlegs / 2); % half the number of legs

pause(5)
for i = 1:xlen
    
    if i >= stepindices(bump_index(1))+1 && upOrDowFlag
        shifty = bump_mag; 
    elseif i >= stepindices(bump_index(1))+1 && i < stepindices(bump_index(2))+1
      shifty = bump_mag; 
    else
      shifty = 0;
    end

    x = xs(i,:);
%       drawnow;
    drawmodel(w, x, [shiftx; shifty], h);
    
%     q1 = x(1);
%     footposn = [shiftx; shifty];
%     hubx = -sin(q1) + footposn(1); huby = cos(q1) + footposn(2);
%     angles = (-hnlegs:hnlegs) *2*alpha;
%     legsx = [sin(-q1 + angles); sin(-q1 + angles + pi)] + hubx;
%     legsy = [cos(-q1 + angles); cos(-q1 + angles + pi)] + huby;
%     htemp = plot(legsx, legsy,'LineWidth',1,'EraseMode','background','Color','Blue'); %original
%     
%     h1 = plot([l_left_corner(1) u_left_corner(1)], [l_left_corner(2) u_left_corner(2)], 'LineWidth',LW,'Color','Black');
%     h2 = plot([u_left_corner(1) u_right_corner(1)],[u_left_corner(2) u_right_corner(2)], 'LineWidth',LW,'Color','Black');
%     h3 = plot([u_right_corner(1) l_right_corner(1)],[u_right_corner(2) l_right_corner(2)], 'LineWidth',LW,'Color','Black');
%     h4 = plot([l_right_corner(1) xlimTemp(2)], [0 0],'Color','Black', 'LineWidth',LW);
    
%     pause(0.02);
%     
%     delete(htemp); %delete(h1); delete(h2); delete(h3); delete(h4);
    
%     clf;
%     pause(0.02)
%     delete(htemp)
    
    
    drawnow;

    M(i) = getframe(gcf);
    writeVideo(vidObj,M(i))

    if find(i == stepindices) 
    %       shiftx = shiftx + 2*sin(alpha);  
      shiftx = shiftx + sin(abs(xs(i,1))) + sin(2*alpha + xs(i,1)); 
    end
end


