
function [xe,te,x,t,energies,indices,vinterms] = onestep3_MStoMS(w, x0, bumps, ctrls, latePushOffFlag)

% The function makes the model walk over the terrain. It will begin solving for the t_SS1, time duration of single
% support 1 then solves the states of single support 1, then do transtion
% (push off-colliison) and then finds the time (t_SS2) and states of the second
% single support of phase

%  Input: w is a walksw2 object, 
%       x0 is state vector (stanleg angle and angular velocity), 
%       bumps is the the terrain heights expressed in angular perturbation of the stance leg, 
%       cntrls are the push off impulses for each step, 
%       latePushOffFlag is the flag to determine whether the push off impulse is preemtptive (before collision) or not

% Output is xe, the mid stance state at the end of the simulation; 
%           te is the time of the end state (mid stance) 
%           t, the time vector; 
%           x, the state vector (stance leg angle and stange leg velocity) over time of the simulation; 
%           energies a structure containing energies (KE, PE, etc.) of system.
%           indices is the mid stance instances of the simulation at the end of each step (each step begins at middle stance the leg is up right and eds at mid stance)
%           vinterms are the velocity of the model after the push off impulse is applied but before the collision happens (Kuo, 2002, Energetics of Actively Powered Locomotion Using the Simplest Walking Model)


% Osman Darici

parms = get(w, 'parms'); 
gamma = parms.gamma; % the slope, it's zero so level terrain walking
alpha = parms.alpha; % half of leg angle. The inter leg angle is 2*alpha and constant. This can be a decision variable if the optimization wants to find a variable alpha that would lead to variable step length
T = 0;               % T is hip torque and always zero 

if isempty(bumps)
    bumps = get(w,'bumps');
end

t = []; x = []; t0 = 0; indices = []; vinterms = [];

for i = 1:length(bumps)
  bump = bumps(i);
  ctrl = ctrls(i);%ctrls(:,i);
  
  if latePushOffFlag
    w = set(w,'latePushOffFlag',1);
  end
 
  % first single stance: begin from mid stance state and run the eq.'s until heel strike
  theta0 = x0(1); thetad0 = x0(2); 
  thetaf = -alpha + bumps(i);% 
  
  t_SS1 = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

  tss1 = linspace(0, t_SS1, 10)';%linspace(0, tf, 20)';   %<-------------10 is hard coded here. This means 10 data points for single support 1

  theta_ss1 = (1/2).*exp((-1).*tss1).*((-1).*((-1)+exp(tss1)).^2.*gamma+((-1)+exp(tss1)).^2.*T+ ...
  theta0+exp(2.*tss1).*theta0+(-1).*thetad0+exp(2.*tss1).*thetad0);

  thetadot_ss1 = (1/2).*exp((-1).*tss1).*(gamma+(-1).*exp(2.*tss1).*gamma+((-1)+exp(2.*tss1)).*T+( ...
  -1).*theta0+exp(2.*tss1).*theta0+thetad0+exp(2.*tss1).*thetad0); %ORIGINAL
      
  % do the switch:
  [xe_temp, energyinfo, vinterm] = s2stransition([theta_ss1(end) thetadot_ss1(end)], w, parms, bump, ctrl);  %ctrl(1)%<---------------------------------
  
  % do the second single stance
  theta0 = xe_temp(1); thetad0 = xe_temp(2); thetaf = 0;
    
  %sol = ode45(@fwalk,[t0 t0+tf],x0(:),options, w, parms, bump);
%   tf = log((-gamma + thetaf - sqrt(thetad0^2 - (theta0 - thetaf)*(-2*gamma + theta0 + thetaf)))/(-gamma + theta0 + thetad0)); %ORIGINAL 
%   tf3 = log((-1).*(theta0+thetad0).^(-1).*((-1).*thetaf+sqrt(2.*gamma.*theta0+(-1).*theta0.^2+thetad0.^2+thetaf.^2)))%%just figuring out the tf qeuation below, do not use this 5 Nov 2020

  t_SS2 = log((-1).*((-1).*gamma+T+theta0+thetad0).^(-1).*(gamma+(-1).*T+(-1).* ...
  thetaf+sqrt(2.*gamma.*theta0+(-2).*T.*theta0+(-1).*theta0.^2+thetad0.^2+ ...
  (-2).*gamma.*thetaf+2.*T.*thetaf+thetaf.^2)));

%   tss2 = linspace(t_SS1(end), t_SS1(end)+t_SS2, 10)';%linspace(0, tf, 20)';
  tss2 = linspace(0, t_SS2, 10)';%linspace(0, tf, 20)';                      %<-------------10 is hard coded here. This means 10 data points for single support 2
 
%   expts = exp(ts); exp2ts = expts.*expts;
%   expmts = exp(-ts);
%   theta = (expmts.*(-(gamma*(-1 + expts).^2) + thetad0*(-1 + exp2ts) + theta0*(1 + exp2ts)))/2.; %ORIGINAL
%   thetadot = (expmts.*(thetad0 + gamma + theta0*(-1 + exp2ts) + thetad0*exp2ts - gamma*exp2ts))/2.; %ORIGINAL
  
  theta_ss2 = (1/2).*exp((-1).*tss2).*((-1).*((-1)+exp(tss2)).^2.*gamma+((-1)+exp(tss2)).^2.*T+ ...
  theta0+exp(2.*tss2).*theta0+(-1).*thetad0+exp(2.*tss2).*thetad0);

  thetadot_ss2 = (1/2).*exp((-1).*tss2).*(gamma+(-1).*exp(2.*tss2).*gamma+((-1)+exp(2.*tss2)).*T+( ...
  -1).*theta0+exp(2.*tss2).*theta0+thetad0+exp(2.*tss2).*thetad0); %ORIGINAL

  xs = [[theta_ss1; theta_ss2] [thetadot_ss1; thetadot_ss2]];
  t = [t; [tss1; tss2+tss1(end)]+ t0]; x = [x; xs]; indices = [indices; length(t)];
   
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm]; 
  
  x0 = xs(end,:); 
  t0 = t(end);
    
  %use this if the hip actuation is constant torque not an impulse
%   energyinfo.hipwork = abs(T)*(xs(1,1)-xs(end,1));
%   energyinfo.hipwork = T*(xs(end,1)-xs(1,1));
  energies(i) = energyinfo; 
end

te = t(end); 
xe = x(end,:); 


 
end % function




