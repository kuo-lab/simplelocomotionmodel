function [xe,te,x,t,energies] = onestep(w, x0, varargin);
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

anim = 0; AbsTol = 1e-9; RelTol = 1e-8; tf = 5.5;

if nargin < 2 | (exist('x0') & isempty(x0)) % x0 not given
  % if xstar exists
  x0 = get(w, 'xstar');
  if isempty(x0)
    error('onestep requires an initial guess x0') 
  end 
end

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
      tf = val;
    otherwise
      error('Onestep options: anim, AbsTol, RelTol, tf')
  end
end

% Integrate the ODE, using m-files or mex-files

% To do this with m-files and without using mex files, use
options = odeset('events',@heelstrikeevent,'AbsTol',AbsTol,'RelTol',RelTol);
sol = ode45(@fwalk,[0 tf],x0(:),options, w);
% the above lines can be replaced with calls to mex files located in the
% private directory

t = sol.x'; x = sol.y';

if anim  % intended for animation
  t = linspace(0, sol.xe, anim)';
  x = deval(sol, t)';
end

te = sol.xe; 
xe = sol.ye'; 

% Now I have to do the switch:
[xe,energies] = s2stransition(xe, w);

if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
end	
