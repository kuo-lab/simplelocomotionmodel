function [grf,grt,cop] = groundreactionforces(t, x, indices, walk)
% GROUNDREACTIONFORCES   Computes ground reaction forces for 2-D rimless
% wheel
% [grf, grt, cop] = groundreactionforces(t, x, walk) computes the ground
% reaction forces in grf, and ground reaction torques in grt, given input
% consisting of time t, state x, and a walk object containing the
% parameters.
% For this model, grf = [fx; fy], the forward horizontal and vertical 
% components.  grt = tz, the moment about the ground3 axis.
% cop = copx, the center of pressure along the forward horizontal axis.

parms = get(walk, 'parms');
gamma = parms.gamma; rgyr = parms.rgyr;

% q1 = x(1);
% u1 = x(2);

sigma2 = 1/(1 + rgyr*rgyr); % this is sigma-squared from McGeer (1990)


teta0 = x(:,1); tetad0 = x(:,2);
u1dot = []; u1dot2 = [];
u1 =  tetad0;

for i = 1:length(indices)
    if i == 1
        firstInd = 1;
    else
        firstInd = indices(i-1)+1;
    end
    tetad0_temp = tetad0(firstInd);
    teta0_temp = teta0(firstInd);
    t_temp = t(firstInd:indices(i),:)-t(firstInd,:); 
    
    u1dot = [u1dot ;(-1/2).*exp(1).^((-1).*t_temp).*((1+exp(1).^(2.*t_temp)).*gamma+(-1).*(1+exp(1).^( ...
  2.*t_temp)).*teta0_temp+tetad0_temp+(-1).*exp(1).^(2.*t_temp).*tetad0_temp)]; %fwalk(t, x, walk); % need state-derivative

% 
%     ts =   t_temp;
%     expts = exp(ts); exp2ts = expts.*expts;
%     expmts = exp(-ts);  expm2ts = expmts.*expmts;
% 
%     u1dot2 = [u1dot2; -0.5*expmts.*(-(1+exp2ts).*teta0_temp + tetad0_temp-exp2ts.*tetad0_temp)]

   
end


% acceleration tangential to com path
% atang = -u1dot; % minus sign is because path is cw
% % acceleration normal to path, along leg
% anorm = u1.*u1; % this is towards ground
% % grf = [cos(x(:,1)) sin(x(:,1)); sin(x(:,1)) -cos(x(:,1))]*[atang; anorm];
% fx = cos(x(:,1)).*atang + sin(x(:,1)).*anorm; 
% fy = sin(x(:,1)).*atang +  -cos(x(:,1)).*anorm;
% grf = [fx fy];

c1 = cos(x(:,1)); s1 = sin(x(:,1));
grf = [ -c1.*u1dot+s1.*(u1.*u1) -s1.*u1dot-c1.*(u1.*u1)+1 ];




grt = 0;
cop = 0;