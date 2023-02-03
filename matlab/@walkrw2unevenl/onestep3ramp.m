
function [xe,te,x,t,energies,indices,vinterms] = onestep3ramp(w, x0, bumps, ctrls, latePushOffFlag, varargin);

%this begins right before HS and hip torque is added

anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5; %<--------------ORIGINAL




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

if isempty(bumps)
    bumps = get(w,'bumps');
end

% if ~isequal(length(bumps), length(ctrls)) % differing size, not sure what to do
%  warning('size of bumps mismatch size of controls');
% end

%SIGN CHANGE TO BEGIN RIGHT BEFORE HEEL STRIKE
% x0(1) = -x0(1);
t = []; x = []; t0 = 0; indices = []; vinterms = [];
% energies = [];
% for i = 1:length(bumps)%size(ctrls, 2) %length(bumps) %<----------
for i = 1:size(ctrls, 2)  
  bump = bumps(i);
  ctrl = ctrls(:,i);
 
  T = 0;
  
  % do not do swothc becasue model is at midstance: 
  if i ~= 1
    [xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl);  %ctrl(1)%<---------------------------------
  else
    xe_temp = x0;
    u1 = xe_temp(2); q1 = xe_temp(1);
    vinterm = [ cos(q1)*(-u1) sin(q1)*(-u1)];

    energyinfo.hipwork = 0;
    energyinfo.before = 0;
    energyinfo.after = 0; 
    energyinfo.intermediate = 0; 
    energyinfo.heelstrikework = 0; 
    energyinfo.pushoffwork = 0; 

    clear u1 q1;
  end
  
  % instead of doing a simulation, let's just plug in some numbers
  theta0 = xe_temp(1); thetad0 = xe_temp(2); %<---------------------------------
  
  if i ~= length(bumps)  %<------CHECK WEATHER THE NEXT STEP HAS A BUMP 
    thetaf = -alpha + bumps(i+1);
  else
%       thetaf = -alpha;
      
      thetaf = 0;%RAMP STUDY
  end
  
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 - thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0)); %ORIGINAL 
  
  tf = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

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
  
  
  
  %use this if the hip actuation is constant torque not an impulse
%   energyinfo.hipwork = abs(T)*(xs(1,1)-xs(end,1));
%   energyinfo.hipwork = T*(xs(end,1)-xs(1,1));

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
