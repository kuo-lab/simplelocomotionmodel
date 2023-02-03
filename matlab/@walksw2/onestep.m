function [xe,te,x,t,energies] = onestep(w, x0, varargin)
% ONESTEP   forward integration of 2-segment, simple walker for one step.
%   [xc,tc,x,t,energies] = onestep(w, x0, flag) integrates the ODE
%   until foot contact occurs, then computes momentum transfer.
%   Input is w (walksw2 object), x0 (state vector).
%   Output is xc, the state following impact; tc, the 
%   time of contact; t, the time vector; x, the state
%   vector over time of the simulation; energies contain the
%   energies before and after push-off and heelstrike.
%   The initial condition x0 = [qstance qswing ustance uswing]
%   and similarly, xc, both being row vectors.
%   Options: 
%     onestep(w, x0, 'anim', Nframes) returns the states
%       evenly spaced in time, with a total of Nframes per step
%     onestep(w, x0, 'AbsTol', 1e-9) specifies the absolute tolerance
%       for the integration, similarly 'RelTol' relative tolerance
%     onestep(w, x0, 'tf', 5.5) specifies the ending time of the
%       simulation.
%   walksw2 is a simplest walker 2-D object

% Arthur D. Kuo, see:
% Kuo, A. D. (2002) Energetics of actively powered locomotion using the 
%   simplest walking model, Journal of Biomechanical Engineering, 124: 113-120. 
% Kuo, A. D. (2001) A simple model predicts the step length-speed 
%   relationship in human walking, Journal of Biomechanical Engineering, 
%   123: 264-269. 


anim = 0; AbsTol = 1e-7; RelTol = 1e-5; tf = 5.5;

if nargin < 2 || (exist('x0') && isempty(x0)) % x0 not given
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
      error('onestep options: anim, AbsTol, RelTol, tf')
  end
end

options = odeset('events',@heelstrikeevent,'AbsTol',AbsTol,'RelTol',RelTol);

% This integrates the ode
sol = ode45(@fwalk,[0 tf],x0(:),options,w,get(w,'parms'));
t = sol.x'; x = sol.y';

if anim  % intended for animation
  t = linspace(0, sol.xe(end), anim+1)'; % linearly interpolate desired number
  t(end) = [];                           % of frames, plus one
  x = deval(sol, t)';
end

te = sol.xe(end)'; 
xe = sol.ye(:,end)';

% Now I have to do the switch:
[xe,energies] = s2stransition(xe, w);%org

%sil**
% [xe1,energies] = s2stransition(xe, w);%P = 0 now fox later********************
% xnew1 = lagrangepushoffsimpwalk2Art(xe, 0)%get(w,'P');
% xnew2 = s2transitionSW2deneme(w, xe, 0);
% [xnew3,lambdas] = heelstrikepwJc(w, xe)
% xe1
% xnew1
% xnew2
% xnew3=xnew3'
%***

if nargout == 0
    plotstep(w,x,t);
end	

