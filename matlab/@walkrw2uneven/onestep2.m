function [xe,te,x,t,energies,indices,vinterms] = onestep2(w, x0, bumps, ctrls, varargin);


%cntrls is added



% ONESTEP   shoots forward integration of 2-segment, simple walker.
%   [xc,tc,x,t,energies] = onestep(w, x0, flag) integrates the ODE
%   until foot contact occurs, then computes momentum transfer.
%   Input is w (walksw2 object), x0 (state vector).
%   Output is xc, the state following impact; tc, the 
%   time of contact; t, the time vector; x, the state
%   vector over time of the simulation; and energies a structure
%   containing energies (KE, PE, etc.) of system.
%   The initial condition x0 = [qstance qswing ustance uswing]
%   and similarly, xc, both being row vectors.
%   Options: 
%     onestep(w, x0, 'anim', Nframes) returns the states
%       evenly spaced in time, with a total of Nframes per step
%     onestep(w, x0, 'AbsTol', 1e-9) specifies the absolute tolerance
%       for the integration, similarly 'RelTol' relative tolerance
%     onestep(w, x0, 'tf', 5.5) specifies the ending time of the
%       simulation.
%   walk2 is an anthropomorphic walker 2-D object

% Arthur D. Kuo, see:
% Kuo, A. D. (2002) Energetics of actively powered locomotion using the 
%   simplest walking model, Journal of Biomechanical Engineering, 124: 113-120. 
% Kuo, A. D. (2001) A simple model predicts the step length-speed 
%   relationship in human walking, Journal of Biomechanical Engineering, 
%   123: 264-269. 

anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5;

% if nargin < 2 | (exist('x0') & isempty(x0)) % x0 not given
%   % if xstar exists
%   x0 = get(w, 'xstar');
%   if isempty(x0)
%     error('onestep requires an initial guess x0') 
%   end 
% end
% 
% bumps can be given either as an explicit argument or as a
% field embedded within the class.
% if nargin < 3 | (exist('bumps') & isempty(bumps)) % bumps not given
%   bumps = get(w,'bumps');
% end
% 
% if nargin < 4 || (exist('ctrls') && isempty(ctrls)) % controls not given
%   ctrls = get(w, 'controls');
% end

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

bumps = get(w,'bumps');%<--------------------

t = []; x = []; t0 = 0; indices = []; vinterms = [];
for i = 1:length(bumps)
  bump = bumps(i);
  ctrl = ctrls(:,i); %<---------------------
  
  if length(ctrl) == 2
    T = ctrl(2);
  else
    T = 0;
  end
  
  w = set(w,'T', T);
  sol = ode45(@fwalk2,[t0 t0+tf],x0(:),options, w, parms, bump);
  % the above lines can be replaced with calls to mex files located in the
  % private directory
  
% Now I have to do the switch:
%   [xe,energyinfo,vinterm] = s2stransition(sol.ye', w, parms, bump);   %ORIGINAL
  
  [xe,energyinfo,vinterm] = s2stransition(sol.ye', w, parms, bump, ctrl);  %<---------------------
  
  t = [t; sol.x']; x = [x; sol.y']; indices = [indices; length(t)];
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm];
  x0 = xe; t0 = t(end);
  
  energyinfo.hipwork = T*(x(end,1)-x(1,1)); %<---------------------
  
  energies(i) = energyinfo; 
end

te = sol.xe; 

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
