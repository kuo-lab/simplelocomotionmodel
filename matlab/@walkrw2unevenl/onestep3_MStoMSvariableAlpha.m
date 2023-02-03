
function [xe,te,x,t,energies,indices,vinterms] = onestep3_MStoMSvariableAlpha(w, x0, bumps, ctrls, latePushOffFlag, nominalStepSpeedRW)


parms = get(w, 'parms'); gamma = parms.gamma; alpha = parms.alpha;

if isempty(bumps)
    bumps = get(w,'bumps');
end

t = []; x = []; t0 = 0; indices = []; vinterms = [];

% c =0.42*(nominalMSspeed)^(0.42-1) = 0.758 %this should be done only and only for nominal walking speed which is 1.25 or 0.3991 dim.less 
c = 0.758;
stepLenNom = 2*sin(alpha);
% nominalStepSpeedRW = 0.478913142607689;

% alphaPrev = alpha;
Vprev = nominalStepSpeedRW;

for i = 1:length(bumps)
  bump = bumps(i);
  ctrl = ctrls(i);%ctrls(:,i);
  
  if latePushOffFlag
    w = set(w,'latePushOffFlag',1);
  end
  T = 0;
  
  % first single stance: begin from mid stance state and run the eq.'s until heel strike
  theta0 = x0(1); thetad0 = x0(2); 
    

  thetaf = -alpha + bumps(i);% if there are N MS speeds before the bump, then there are N po's and N bumps, -3, -2,-,1,0, 1,2,3 for N = 7, 7 bumps & 7 po's, 3 before and 3 after
  
  t_SS1 = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

  tss1 = linspace(0, t_SS1, 10)';%linspace(0, tf, 20)';

  theta_ss1 = (1/2).*exp((-1).*tss1).*((-1).*((-1)+exp(tss1)).^2.*gamma+((-1)+exp(tss1)).^2.*T+ ...
  theta0+exp(2.*tss1).*theta0+(-1).*thetad0+exp(2.*tss1).*thetad0);

  thetadot_ss1 = (1/2).*exp((-1).*tss1).*(gamma+(-1).*exp(2.*tss1).*gamma+((-1)+exp(2.*tss1)).*T+( ...
  -1).*theta0+exp(2.*tss1).*theta0+thetad0+exp(2.*tss1).*thetad0); %ORIGINAL
      
  % do the switch:
  [xe_temp, energyinfo, vinterm] = s2stransition([theta_ss1(end) thetadot_ss1(end)], w, parms, bump, ctrl);  %ctrl(1)%<---------------------------------
  
  stepLenCurrent = abs(sin(theta_ss1(end))) + abs(sin(xe_temp(1))); %<-------UPDATE the step length
  
  % do the second single stance
  theta0 = xe_temp(1); thetad0 = xe_temp(2); thetaf = 0;
    
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 - thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0)); %ORIGINAL 
%   tf3 = log((-1).*(theta0+thetad0).^(-1).*((-1).*thetaf+sqrt(2.*gamma.*theta0+(-1).*theta0.^2+thetad0.^2+thetaf.^2)))%%just figuring out the tf qeuation below, do not use this 5 Nov 2020

  t_SS2 = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

%   tss2 = linspace(t_SS1(end), t_SS1(end)+t_SS2, 10)';%linspace(0, tf, 20)';
  tss2 = linspace(0, t_SS2, 10)';%linspace(0, tf, 20)';
 
%   expts = exp(ts); exp2ts = expts.*expts;
%   expmts = exp(-ts);
%   theta = (expmts.*(-(gamma*(-1 + expts).^2) + thetad0*(-1 + exp2ts) + theta0*(1 + exp2ts)))/2.; %ORIGINAL
%   thetadot = (expmts.*(thetad0 + gamma + theta0*(-1 + exp2ts) + thetad0*exp2ts - gamma*exp2ts))/2.; %ORIGINAL
  
  theta_ss2 = (1/2).*exp((-1).*tss2).*((-1).*((-1)+exp(tss2)).^2.*gamma+((-1)+exp(tss2)).^2.*T+ ...
  theta0+exp(2.*tss2).*theta0+(-1).*thetad0+exp(2.*tss2).*thetad0);

  thetadot_ss2 = (1/2).*exp((-1).*tss2).*(gamma+(-1).*exp(2.*tss2).*gamma+((-1)+exp(2.*tss2)).*T+( ...
  -1).*theta0+exp(2.*tss2).*theta0+thetad0+exp(2.*tss2).*thetad0); %ORIGINAL

  xs = [[theta_ss1; theta_ss2] [thetadot_ss1; thetadot_ss2]];
  
  stepTime = [tss1; tss2+tss1(end)];
  t = [t; stepTime + t0]; x = [x; xs]; indices = [indices; length(t)];
%   t = [t; [tss1; tss2+tss1(end)]+ t0]; x = [x; xs]; indices = [indices; length(t)];
   
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm]; 
  
  x0 = xs(end,:); 
  t0 = t(end);
  
   %new alpha for the next step. uses this step's length stepLenCurrent is ALREADY UPDATED
   Vnew = stepLenCurrent/stepTime(end);%this is step length/ step duration
%    alpha = asin( (stepLenCurrent + c*(Vnew - Vprev) ) /2 );%ORGINAL %<-----------------------------------------
    
%     alpha = asin( (stepLenNom + c*(Vnew - Vprev) ) /2 ); %trying

    midstanceVel_curr = abs(thetadot_ss2(end));
    alpha = asin( ( stepLenNom  + c*( midstanceVel_curr-Vprev ) ) /2 );
   
%     Vprev = Vnew; %ORGINAL, do not update to try %<-----------------------------------------
    
    
%     OLD VERSION
%    alphaNew = asin( ( 2*sin(alphaBefore) + c*( abs(midstanceVel_curr)-abs(nomMidStanceSpeedPrev) ) ) /2 );   
   %     alphaNew = asin( ( 2*sin(alphaBefore) + c*( abs(midstanceVel_curr)-abs(nomMidStanceSpeedPrev) ) ) /2 );
   % new alpha is based on new step length. NewStepLength = oldStepLength + c *(midstanceVel_curr-nomMidStanceSpeedPrev)
   % here c is the slope of s = v^0.42 nominal step len-speed curve. Basically it is 1st order approx. (taylor series) stepLen = s(Vnom) + c * deltaV. Therefore c =0.42*(Vnom)^(0.42-1) 1st derivatie
  
       %IMPORTANT store prev alpha and mid stance vel
    %if you dont update these 2, they will stay at nominal and smiliar to Art's
%       alphaBefore = alphaNew;
%       nomMidStanceSpeedPrev = midstanceVel_curr; 
   
   
   
  %use this if the hip actuation is constant torque not an impulse
%   energyinfo.hipwork = abs(T)*(xs(1,1)-xs(end,1));
%   energyinfo.hipwork = T*(xs(end,1)-xs(1,1));
  energies(i) = energyinfo; 
end

te = t(end); 
xe = x(end,:); 


 
end % function




