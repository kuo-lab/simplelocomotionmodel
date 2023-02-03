
function [xe,te,x,t,energies,indices,vinterms] = onestep5_v2(w, stepTimeNomRW, varargin)

%this to find numerical derivatives

anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5; %<--------------ORIGINAL
latePushOffFlag = 0;

parms = get(w, 'parms'); gamma = parms.gamma; alpha = parms.alpha;
ctrls = get(w, 'P');
bumps = 0;
% stepTimeNomRW = 1.7227;%<-------------
ys = [get(w, 'xstar') stepTimeNomRW ]
ys(1) = -ys(1);
xState = ys;


opt_argin = varargin;
while length(opt_argin) >= 2,
  opt = opt_argin{1};
  val = opt_argin{2};
  opt_argin = opt_argin(3:end);
  switch opt
    case 'state'
      xState = val;
    case 'bump'
      bumps = val;
    case 'pushoff'
      ctrls = val; % here tf is defined as the time beyond the start of this step
    otherwise
      error('Onestep options: anim, AbsTol, RelTol, tf')
  end
end

t = []; x = []; %t0 = 0; 
indices = []; vinterms = [];

x0 = xState(:,1:2); %t0 = xState(3); %t0dot = xState(4);



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
        w = set(w,'latePushOffFlag',1);
    end
  end
  
  if length(ctrl) == 2
    T = ctrl(2);
  else
    T = 0;
  end
  
  % begin with the switch: 
  [xe_temp, energyinfo, vinterm] = s2stransitionTemp(x0, w, parms, 0, ctrl(1));   %this returns x(1) differently 2*alpha+x(1)
  
  % instead of doing a simulation, let's just plug in some numbers
  theta0 = xe_temp(1); thetad0 = xe_temp(2); %<---------------------------------
  
  thetaf = -alpha + bumps(i);
  
%   if i ~= length(bumps)  %<------CHECK WEATHER THE NEXT STEP HAS A BUMP 
%     thetaf = -alpha + bumps(i+1);
%   else
%       thetaf = -alpha;
%   end
  
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
  t = [t; ts]; %<-----------ts+t0

  x = [x; xs]; 
  indices = [indices; length(t)];
  
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm]; 
  
  x0 = xs(end,:); %<---------------------------------  
  t0 = t(end);
%   energyinfo.hipwork = abs(T)*(xs(1,1)-xs(end,1));
  energyinfo.hipwork = T*(xs(end,1)-xs(1,1));
  energies(i) = energyinfo; 
end


% te = t(end); 
xe = [xs(end,:) t(end) ]'; %<-------transpose


if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
   xlabel('time'); ylabel('states');
end	

if ~(all(isreal(ts)) | all(all(isreal(xs))))
  xe = [NaN NaN];
  te = NaN;
  
end %if


end % function


