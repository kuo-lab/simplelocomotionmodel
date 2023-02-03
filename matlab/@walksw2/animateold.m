function animate(w,varargin)
% animate(w) 
%   animates the simplest walking model described by w.
% animate(w, 'numsteps', N) automatically repeats the simulation N times (default 2).
% animate(w, x, stepindices) animates w with the given states given in x
%   (rows of the state vector), and with stepindices containing a list
%   of the number of frames in each step.
% animate(w, 'arrows', 1) also draws arrows for COM velocity
% animate(w, 'save', 1) animates w and saves each frame as an 
%   adobe illustrator file
% animate(w, 'frames', Nframes) draws a certain number of frames per step

% Art Kuo

L = 1; R = 0;

footn = 1; % how many segments to draw in the curved foot

debg = 0;

numsteps = 3; saveflag = 0; arrowflag = 0; nframes = 16; % default values
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
    elseif size(secondargument,1) > 1 && size(secondargument,2) == 4 % it's a matrix of states
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

if length(stepindices) == 1 % there's just one step stored in xs
  stepindices = [xlen repmat(xlen-1, 1, numsteps-1)];
  startindex = repmat(1,numsteps,1); % extra frame
  endindex = repmat(xlen, numsteps, 1);
elseif length(stepindices) > 1  % there's more than one step in xs
  endindex = cumsum(stepindices);
  startindex = [1 endindex+1];
end
% Now numsteps contains the number of steps, stepindices
% contains the number of frames in each step, and
% startindex and endindex contain indices for each step

% Estimate range of walking
distance = 2*numsteps*R*xs(1,1)+(numsteps+1)*(2*(L-R)*abs(sin(xs(1,1))));
xlimit = [-R-0.1 distance+R+0.1]-(L-R)*abs(sin(xs(1,2))); 
ylimit = [-0.05 1.35]*L;

% arrow parameters
aang = pi/6; scale = 0.02; scale2 = 2; vx2 = 0.4; vy2 = 1.2;

% A foot
alpha = max(xs(:,1))*1.1; % the range of the curved foot
% foot starts at -sin(a),cos(a)
% and goes to sin(a),cos(a)
footang = linspace(-alpha*1.1, alpha*1.1, footn);
footxy = R*[sin(footang); -cos(footang)];

% Initialize
clf; 
th1 = xs(1,1); th2 = xs(1,1)-xs(1,2); u1 = xs(1,3);
contactpoint = -th1*R;
Rot1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
Rot2 = [cos(th2) -sin(th2); sin(th2) cos(th2)];

footx1 = Rot1(1,:)*footxy + contactpoint; footy1 = Rot1(2,:)*footxy + R;
legsxy = [0  -sin(th1)  -sin(th1)+sin(th2);
  0   cos(th1)   cos(th1)-cos(th2)];
legsx = legsxy(1,:) + contactpoint + R*sin(th1);
legsy = legsxy(2,:) + R - R*cos(th1);       
footx2 = Rot2(1,:)*footxy + legsx(3) - R*sin(th2);
footy2 = Rot2(2,:)*footxy + legsy(3) + R*cos(th2);
pcm = legsxy(:,2) + [contactpoint+R*sin(th1);R-R*cos(th1)];
vcm = [-u1*(R + (L-R)*cos(th1)); -u1*(L-R)*sin(th1)];
velang = atan2(vcm(2),vcm(1));
velx = [0 vcm(1) vcm(1)-scale*cos(velang+aang) NaN vcm(1) vcm(1)-scale*cos(velang-aang)]+pcm(1);
vely = [0 vcm(2) vcm(2)-scale*sin(velang+aang) NaN vcm(2) vcm(2)-scale*sin(velang-aang)]+pcm(2);
velx2 = scale2*[0 vcm(1) vcm(1)-scale*cos(velang+aang) NaN vcm(1) vcm(1)-scale*cos(velang-aang)]+vx2;
vely2 = scale2*[0 vcm(2) vcm(2)-scale*sin(velang+aang) NaN vcm(2) vcm(2)-scale*sin(velang-aang)]+vy2;

set(gcf, 'color', [1 1 1]); set(gca,'DataAspectRatio',[1 1 1],'Visible','off','NextPlot','Add','XLim',xlimit,'YLim',ylimit);

%hf1 = line(footx1,footy1,'LineWidth',3,'EraseMode','normal'); 
%hf2 = line(footx2,footy2,'LineWidth',3,'EraseMode','normal');
hlegs = line(legsx,legsy,'LineWidth',3,'EraseMode','background');
hvel = line(velx,vely,'color','m','LineWidth',2,'EraseMode','background');
hpelv = plot(legsx(2),legsy(2),'.','MarkerSize',30,'EraseMode','background');
hgnd = line(xlimit,[0 0]-.01,'color',[0 0 0],'linewidth',2);
%hvel2 = line(velx2,vely2,'color','m','LineWidth',2,'EraseMode','background');

th1old = th1; cntr = 1;
for j = 1:numsteps
  for i = startindex(j):endindex(j)
    th1 = xs(i,1); th2 = xs(i,1) - xs(i,2);
    contactpoint = contactpoint - (th1-th1old)*R; % roll forward a little
    th1old = th1;
    Rot1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
    Rot2 = [cos(th2) -sin(th2); sin(th2) cos(th2)];
    
    footx1 = Rot1(1,:)*footxy + contactpoint; footy1 = Rot1(2,:)*footxy + R;
    legsxy = [0  -sin(th1)  -sin(th1)+sin(th2);
              0   cos(th1)   cos(th1)-cos(th2)];
    legsx = legsxy(1,:) + contactpoint + R*sin(th1);
    legsy = legsxy(2,:) + R - R*cos(th1);       
    footx2 = Rot2(1,:)*footxy + legsx(3) - R*sin(th2);
    footy2 = Rot2(2,:)*footxy + legsy(3) + R*cos(th2);
    
    pcm = legsxy(:,2) + [contactpoint+R*sin(th1);R-R*cos(th1)];
    vcm = [-u1*(R + (L-R)*cos(th1)); -u1*(L-R)*sin(th1)];
    velang = atan2(vcm(2),vcm(1));
    velx = [0 vcm(1) vcm(1)-scale*cos(velang+aang) NaN vcm(1) vcm(1)-scale*cos(velang-aang)]+pcm(1);
    vely = [0 vcm(2) vcm(2)-scale*sin(velang+aang) NaN vcm(2) vcm(2)-scale*sin(velang-aang)]+pcm(2);
    velx2 = scale2*[0 vcm(1) vcm(1)-scale*cos(velang+aang) NaN vcm(1) vcm(1)-scale*cos(velang-aang)]+vx2;
    vely2 = scale2*[0 vcm(2) vcm(2)-scale*sin(velang+aang) NaN vcm(2) vcm(2)-scale*sin(velang-aang)]+vy2;
    
%    set(hf1,'Xdata',footx1,'Ydata',footy1);
%    set(hf2,'Xdata',footx2,'Ydata',footy2);
    set(hlegs,'Xdata',legsx,'Ydata',legsy);
    set(hvel,'Xdata',velx,'Ydata',vely);
    set(hpelv,'Xdata',legsx(2),'Ydata',legsy(2));
    %set(hvel2,'Xdata',velx2,'Ydata',vely2);
    if 0  % display secondary velocity?
    if i==1 & j > 1  % stick velocity arrow
      hveli=line(velx2,vely2,'color','m','LineWidth',2,'erasemode','none');
      oldx = get(hvelo,'xdata'); oldy = get(hvelo,'ydata');
      hsang = atan2(vely2(2)-oldy(2),velx2(2)-oldx(2));
      velxh = [oldx(2) velx2(2) velx2(2)-scale2*scale*cos(hsang+aang) NaN velx2(2) velx2(2)-scale2*scale*cos(hsang-aang)];
    	velyh = [oldy(2) vely2(2) vely2(2)-scale2*scale*sin(hsang+aang) NaN vely2(2) vely2(2)-scale2*scale*sin(hsang-aang)];  
  		hvelhs = line(velxh,velyh,'color','r','Linewidth',2,'erasemode','none');    
    end
    end
    drawnow; pause(0.05)
    if saveflag
      print('-dill',sprintf('walk%02d',cntr));
    end
    if debg, pause, xs(i,:), j, end;
    cntr = cntr + 1;
  end
  contactpoint = contactpoint - (L-R)*(sin(th1)-sin(th2)); th1old = th2;
	%hvelo = line(velx2,vely2,'color','m','LineWidth',2);
end


% Have to deal with two situations:
% one step that is repeated many times
% or multiple steps that don't have to be identical