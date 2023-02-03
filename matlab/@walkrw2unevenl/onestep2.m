function [xe,te,x,t,energies,indices,vinterms] = onestep2(w, x0, bumps, ctrls, varargin);

%onestep2 is exactly same with onestep except some argumnet options
%commented out. You have onestep2 for linear model and onestep2 for
%non-linear model so you can run the same script to switch between the
%models. Onestep3 is your originla file with begining from right before HS
%and hip torque

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

% if nargin < 2 || (exist('x0') && isempty(x0)) % x0 not given
%   % if xstar exists
%   x0 = get(w, 'xstar');
%   if isempty(x0)
%     error('onestep requires an initial guess x0') 
%   end 
% end

% bumps can be given either as an explicit argument or as a
% field embedded within the class.
% if nargin < 3 || (exist('bumps') && isempty(bumps)) % bumps not given
%   bumps = get(w,'bumps');
% end
% 
% if nargin < 4 || (exist('ctrls') && isempty(ctrls)) % controls not given
%   ctrls = get(w, 'controls');
% end
% 
% if isempty(ctrls) || any(any(isnan(ctrls)))
%   ctrls = [get(w,'P'); 0];
% end

% try to reconcile the bumps and controls, with bumps taking precedence
% if size(ctrls,2)==1 && ~isscalar(bumps) % only given one control, so make it match bumps
%   ctrls = repmat(ctrls, 1, size(bumps,2));
% end
% 
% if size(ctrls,1)==1 % no hip given, so set it to zero
%   ctrls(2,:) = 0;
% end

%if ~isequal(size(bumps), size(ctrls)) % differing size, not sure what to do
%  warning('size of bumps mismatch size of controls');
%end

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
%options = odeset('events',@heelstrikeevent,'AbsTol',AbsTol,'RelTol',RelTol);
parms = get(w, 'parms'); gamma = parms.gamma; alpha = parms.alpha;

bumps = get(w,'bumps');%<--------------------

t = []; x = []; t0 = 0; indices = []; vinterms = [];  
for i = 1:size(ctrls, 2)%length(bumps)
  bump = bumps(i);
  ctrl = ctrls(:,i);
  
%   if length(ctrl) == 2
%     T = ctrl(2);
%   else
%     T = 0;
%   end
%   
    T = ctrl(2); %there is always hip torque, but iniate it with zero
  
  % instead of doing a simulation, let's just plug in some numbers
  theta0 = x0(1); thetad0 = x0(2);
  thetaf = -alpha + bump;
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
  
  %art
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 -thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0));%   ORIGINAL 
    
   %Osman from mathematica  
%   tfX = log(  (   -gamma + T + thetaf - sqrt( thetad0^2 - (theta0 -thetaf)*(-2*gamma + theta0 + thetaf) -2*T*theta0 + 2*T*thetaf)   )/(-gamma + T + theta0 + thetad0)   );%
  
    %Osman COPIED from mathematica
  tf = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));
  

  
  if nargout == 0 || nargout > 2 % either want a plot or want xs and ts
    ts = linspace(0, tf, 20)';
  else
    ts = tf;
  end
  expts = exp(ts); exp2ts = expts.*expts;
  expmts = exp(-ts);
  
  %Art
%   theta = (expmts.*(-(gamma*(-1 + expts).^2) + thetad0*(-1 + exp2ts) + theta0*(1 + exp2ts)))/2.;
%   thetadot = (expmts.*(thetad0 + gamma + theta0*(-1 + exp2ts) + thetad0*exp2ts - gamma*exp2ts))/2.;

  %Osman from mathematica THIS IS NOT TRUE. 
%  thetaX = (expmts.*(-(gamma*(-1 + expts).^2) + T*(-1 + expts).^2 + thetad0*(-1 + exp2ts) + theta0*(1 + exp2ts)))/2.;
%  thetadotX = (expmts.*(thetad0 + gamma + theta0*(-1 + exp2ts) + thetad0*exp2ts - gamma*exp2ts) + T * (-1+exp2ts))/2.;
  
 %Osman COPIED from mathematica THIS IS TRUE, because you compared linear
 %and non-linear model and they are almost same


    theta = (1/2).*exp((-1).*ts).*((-1).*((-1)+exp(ts)).^2.*gamma+((-1)+exp(ts)).^2.*T+ ...
  theta0+exp(2.*ts).*theta0+(-1).*thetad0+exp(2.*ts).*thetad0);

  thetadot = (1/2).*exp((-1).*ts).*(gamma+(-1).*exp(2.*ts).*gamma+((-1)+exp(2.*ts)).*T+( ...
  -1).*theta0+exp(2.*ts).*theta0+thetad0+exp(2.*ts).*thetad0);

  xs = [theta thetadot];
% Now I have to do the switch:
%  [xe,energyinfo,vinterm] = s2stransition(sol.ye', w, parms, bump);
  [xe,energyinfo,vinterm] = s2stransition(xs(end,:), w, parms, bump, ctrl(1)); 
  t = [t; ts+t0]; x = [x; xs]; indices = [indices; length(t)];
 
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm];
  x0 = xe; t0 = t(end);
  
  energyinfo.hipwork = abs(T*(x(end,1)-x(1,1))); %<---------------------
  
  energies(i) = energyinfo; 
end

te = t(end); 

if anim  % intended for animation, only works for one step!
  if length(bumps) > 1
    warning('animation not meant for multiple steps!');
  end
  
  % for linearized pendulum, no need to do anything because
  % time steps already equally spaced
  %t = linspace(0, sol.xe, anim)';
  %x = deval(sol, t)';
end

if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
   xlabel('time'); ylabel('states');
end	

if ~(all(isreal(ts)) | all(all(isreal(xs))))
  xe = [NaN NaN];
  te = NaN;
  
end %if


end % function


