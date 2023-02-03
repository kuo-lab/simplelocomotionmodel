function [xe,te,x,t,energies,indices,vinterms] = onestep_NonlinforRw2Paper(w, x0, bumps, ctrls, latePushOffFlag, varargin);


%cntrls is added


% anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5;
anim = 0; AbsTol = 1e-12; RelTol = 1e-10; tf = 5.5;

opt_argin = varargin;
while length(opt_argin) >= 2,
  opt = opt_argin{1};
  val = opt_argin{2};
  opt_argin = opt_argin(3:end);
  switch opt
    case 'anim'
      anim = val;
    case 'AbsTol'
      AbsTol = val;
    case 'RelTol'
      RelTol = val;
    case 'tf'
      tf = val; % here tf is defined as the time beyond the start of this step
    otherwise
      error('Onestep options: anim, AbsTol, RelTol, tf')
  end
end

% Integrate the ODE, using m-files or mex-files

% To do this with m-files and without using mex files, use
options = odeset('events',@heelstrikeevent,'AbsTol',AbsTol,'RelTol',RelTol);
parms = get(w, 'parms');

if isempty(bumps)
    bumps = get(w,'bumps');
end

%SIGN CHANGE TO BEGIN RIGHT BEFORE HEEL STRIKE
x0(1) = -x0(1); %you dont need to actually because the the angle after transition does not depend on this angle

t = []; x = []; t0 = 0; indices = []; vinterms = [];
for i = 1:size(ctrls, 2) 
  bump = bumps(i);
  ctrl = ctrls(:,i);
  
  if latePushOffFlag
    bumpIndexes = find(bumps ~= 0); 
    if  bumpIndexes
        if  (i == bumpIndexes(1)) %|| (i == bumpIndexes(2))
            w = set(w,'latePushOffFlag',1);
        else
            w = set(w,'latePushOffFlag',0);
        end
    else
%         w = set(w,'latePushOffFlag',1);
%         error('always late push off are you sure?')
        
        %this is a special condition walking over zero bump with first step
%         late push off
%         keyboard();
        if  (i == 1) %|| (i == bumpIndexes(2))
            w = set(w,'latePushOffFlag',1);
        else
            w = set(w,'latePushOffFlag',0);
        end
    end
  end
  
  if length(ctrl) == 2
    T = ctrl(2);
  else
    T = 0;
  end
  
    % begin with the switch: 
  [xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl(1));  %ctrl(1)%<---------------------------------
    
  theta0 = xe_temp(1); thetad0 = xe_temp(2); %<---------------------------------
  
  w = set(w,'T', T);
  
  %you should switch ot the next bump for HS event
  if i ~= length(bumps)  %<------CHECK WEATHER THE NEXT STEP HAS A BUMP 
    bumpNext = bumps(i+1);
  else
    bumpNext = 0;
  end
  
  sol = ode45(@fwalk2,[t0 t0+tf],[theta0  thetad0],options, w, parms, bumpNext);
  % the above lines can be replaced with calls to mex files located in the
  % private directory
  
% % Now I have to do the switch:
% %   [xe,energyinfo,vinterm] = s2stransition(sol.ye', w, parms, bump);   %ORIGINAL
%   
%   [xe,energyinfo,vinterm] = s2stransition(sol.ye', w, parms, bump, ctrl);  %<---------------------
%   
  t = [t; sol.x']; x = [x; sol.y']; indices = [indices; length(t)];
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm];
  x0 = sol.ye;
  t0 = sol.xe;
  
  energyinfo.hipwork = T*(x(end,1)-x(1,1)); %<---------------------
  
  energies(i) = energyinfo; 
end

te = t(end); 
xe = x(end,:); %

if anim  % intended for animation, only works for one step!
  if length(bumps) > 1
    warning('animation not meant for multiple steps!');
  end
  t = linspace(0, sol.xe, anim)';
  x = deval(sol, t)';
end

if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
   xlabel('time'); ylabel('states');
end	
