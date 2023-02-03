
function [xe,te,x,t,energies,indices,vinterms] = onestep3Animate(w, x0, bumps, ctrls, latePushOffFlag, timeEnd, indicesEnd, varargin);

%tHIS FILE DOES NOT WORK NOW JUST FOR ANIMATION


%this begins right before HS and hip torque is added

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

anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5; %<--------------ORIGINAL


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

% if nargin < 4 || (exist('ctrls') && isempty(ctrls)) % controls not given
%   ctrls = get(w, 'controls');
% end

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


parms = get(w, 'parms'); gamma = parms.gamma; alpha = parms.alpha;
bumps = get(w,'bumps');

%SIGN CHANGE TO BEGIN RIGHT BEFORE HEEL STRIKE
x0(1) = -x0(1);

t = []; x = []; t0 = 0; indices = []; vinterms = [];  

timeVec = timeEnd/indicesEnd:timeEnd;

for i = 1:length(bumps)%size(ctrls, 2) %length(bumps) %<----------
    
  bump = bumps(i);
  ctrl = ctrls(:,i);
  
  if latePushOffFlag
    bumpIndexes = find(bumps ~= 0); 
    if  bumpIndexes
        if  (i == bumpIndexes(1)) || (i == bumpIndexes(2))
            w = set(w,'latePushOffFlag',1);
        else
            w = set(w,'latePushOffFlag',0);
        end
    else
        w = set(w,'latePushOffFlag',1);
    end
  end
  
  if length(ctrl) == 2
    T = ctrl(2);
  else
    T = 0;
  end
%   T = ctrl(2); %there is always hip torque, but iniate it with zero
  
  % begin with the switch: 
  [xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl(1));  %ctrl(1)%<---------------------------------

  % instead of doing a simulation, let's just plug in some numbers
  theta0 = xe_temp(1); thetad0 = xe_temp(2); %<---------------------------------
  
  if i ~= length(bumps)  %<------CHECK WEATHER THE NEXT STEP HAS A BUMP 
    thetaf = -alpha + bumps(i+1);
  else
      thetaf = -alpha;
  end
  
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 - thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0)); %ORIGINAL 
  
%   tf = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
%   thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
%   (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

    
%    timeVec()

  if nargout == 0 || nargout > 2 % either want a plot or want xs and ts
    ts = linspace(0, tf, 20)';%linspace(0, tf, 20)';
  else
    ts = tf;
  end
  
%   expts = exp(ts); exp2ts = expts.*expts;
%   expmts = exp(-ts);
%   theta = (expmts.*(-(gamma*(-1 + expts).^2) + thetad0*(-1 + exp2ts) + theta0*(1 + exp2ts)))/2.; %ORIGINAL
%   thetadot = (expmts.*(thetad0 + gamma + theta0*(-1 + exp2ts) + thetad0*exp2ts - gamma*exp2ts))/2.; %ORIGINAL
 

    theta = (1/2).*exp((-1).*ts).*((-1).*((-1)+exp(ts)).^2.*gamma+((-1)+exp(ts)).^2.*T+ ...
  theta0+exp(2.*ts).*theta0+(-1).*thetad0+exp(2.*ts).*thetad0);

  thetadot = (1/2).*exp((-1).*ts).*(gamma+(-1).*exp(2.*ts).*gamma+((-1)+exp(2.*ts)).*T+( ...
  -1).*theta0+exp(2.*ts).*theta0+thetad0+exp(2.*ts).*thetad0);


  
  xs = [theta thetadot];

  
  t = [t; ts+t0]; x = [x; xs]; indices = [indices; length(t)];
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm];
  
  x0 = xs(end,:); %<---------------------------------  
  t0 = t(end);
  energyinfo.hipwork = abs(T)*(xs(1,1)-xs(end,1));
  energies(i) = energyinfo; 
end

te = t(end); 
xe = xs(end,:); %<---------------------------------

if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
   xlabel('time'); ylabel('states');
end	

if ~(all(isreal(ts)) | all(all(isreal(xs))))
  xe = [NaN NaN];
  te = NaN;
  
end %if


end % function



%if it begins on a bump %<---------------------
% if bumps(1) ~= 0 
%     firstStepOnBump = 1;
% else
%     firstStepOnBump = 0;
% end



% this step can be called in the main script
%   if firstStepOnBump == 1%if it begins on bump
%       w_temp = walkrw2unevenl();
%       w_temp = set(w_temp,'bumps', [0 bumps(1)]); %remember to shift the bump here to get the last state before the bump
%       [xc_temp ,tc_temp ,xs_temp ,ts_temp ,energies_temp ,indices_temp ] = onestep2(w_temp,[],[],[],0);
%       x0 = xs_temp(indices_temp(1),:);   %becareful the first index is the on bump state
%       clear xc_temp tc_temp xs_temp ts_temp energies_temp indices_temp
%       firstStepOnBump = 0;
%   end
