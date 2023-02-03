
function [xe,te,x,t,energies,indices,vinterms, alphaNewAll] = onestep3variableAlpha(w, x0, bumps, ctrls, latePushOffFlag, nomMidStanceSpeed, varargin);

%this m file is to answer reviewers of eLife they asked about varying step
%length. We will vary alpha based on previous mdi stance velocity

%this begins right before HS and hip torque is added
% this equations assumes sigma is 1 look at Art' s notes if you change rgyr, g or L*

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
x0(1) = -x0(1);
t = []; x = []; t0 = 0; indices = []; vinterms = [];

clear theta thetadot 
thetaf = 0;%mid stance angle
alphaBefore = alpha;
% for i = 1:length(bumps)%size(ctrls, 2) %length(bumps) %<----------

% load nominalMidStanceVel_1_50hallway
% midstanceVel_before2 = nominalMidStanceVel_1_50hallway;

nomMidStanceSpeedPrev = nomMidStanceSpeed;
% c =0.42*(nominalMSspeed)^(0.42-1) = 0.758 %this should be done only and only for nominal walking speed which is 1.25 or 0.3991 dim.less 
c = 0.758;

alphaNewAll = [];

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
        error('check this, are you sure you want this?, you need to set the i = XXX below')
%         
%         this is a special condition walking over zero bump with first step
%         late push off
        keyboard();
        if  (i == 2) %|| (i == bumpIndexes(2)) %change this number manually depending on what step with zero bump you want the PO late
            w = set(w,'latePushOffFlag',1);
        else
            w = set(w,'latePushOffFlag',0);
        end
    end
  end
  
  %ORIGINAL%<---------------------------------
%   if length(ctrl) == 2
%     T = ctrl(2);
%   else
%     T = 0;
%   end
    T = 0;
  
    %begin with the switch: 
    %[xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl(1));  %ORIGINAL
    [xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl);  %ctrl(1)%<---------------------------------

    % instead of doing a simulation, let's just plug in some numbers
    theta0 = xe_temp(1); thetad0 = xe_temp(2); %<---------------------------------

    %Here we need to have the midstance veloctiy
    thetaf_mid = 0;%this is mid stance angle
    timeMS_temp  = log( (thetaf_mid - sqrt(thetaf_mid^2 - theta0^2 +  thetad0^2)) / (theta0+thetad0) );     
    midstanceVel_curr = (1/2).* (  exp(-timeMS_temp).* (thetad0-theta0) + exp(timeMS_temp).* (thetad0+theta0) );   

    %new alpha
   
    alphaNew = asin( ( 2*sin(alphaBefore) + c*( abs(midstanceVel_curr)-abs(nomMidStanceSpeedPrev) ) ) /2 ); 
    % new alpha is based on new step length. NewStepLength = oldStepLength + c *(midstanceVel_curr-nomMidStanceSpeedPrev)
    % here c is the slope of s = v^0.42 nominal step len-speed curve. Basically it is 1st order approx. (taylor series) stepLen = s(Vnom) + c * deltaV. Therefore c =0.42*(Vnom)^(0.42-1) 1st derivatie

    
    %store prev alpha and mid stance vel
    %if you dont update these 2, they will stay at nominal and smiliar to Art's
%       alphaBefore = alphaNew;
%       nomMidStanceSpeedPrev = midstanceVel_curr; 

    w = set(w, 'alpha', alphaNew);
    alpha = alphaNew;% we should new alpha below
    alphaNewAll = [alphaNewAll alphaNew];

  if i ~= length(bumps)  %<------CHECK WEATHER THE NEXT STEP HAS A BUMP 
     thetaf = -alpha + bumps(i+1);
  else
      thetaf = -alpha;
  end
  
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 - thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0)); %ORIGINAL 
%   tf3 = log((-1).*(theta0+thetad0).^(-1).*((-1).*thetaf+sqrt(2.*gamma.*theta0+(-1).*theta0.^2+thetad0.^2+thetaf.^2)))%%just figuring out the tf qeuation below, do not use this 5 Nov 2020

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
  -1).*theta0+exp(2.*ts).*theta0+thetad0+exp(2.*ts).*thetad0); %ORIGINAL

%     thetadotX = (1/2).*exp((-1).*ts).*((-1).*theta0+exp(2.*ts).*theta0+thetad0+exp(2.*ts).*thetad0);%checking the ORIGINAL euqtion above line


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
