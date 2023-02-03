function [grf,grt,cop] = groundreactionforcesMStoMS(t, x, indices, walk)


%the ground reaction forces are basically formed by tangetntial and radial
%accelerations that are found for single support 1 and single  supoort 2.

%  Input: 
%           t, the time vector; 
%           x, the state vector (stance leg angle and stange leg velocity) over time of the simulation; 
%           indices is the mid stance instances of the simulation at the end of each step (each step begins at middle stance the leg is up right and eds at mid stance)
%           walk is a walksw2 object, 

% output grf: ground reaction force
        % grt: ground reaction torque
        % cop: center of pressure

parms = get(walk, 'parms');
gamma = parms.gamma; rgyr = parms.rgyr;


teta0 = x(:,1); tetad0 = x(:,2);
u1dot1 = []; u1dot2 = []; u1dot = [];
u1 =  tetad0;

for i = 1:length(indices)
    if i == 1
        firstInd = 1;
    else
        firstInd = indices(i-1)+1;
    end
    tetad0_temp = tetad0(firstInd);
    teta0_temp = teta0(firstInd);
    t_temp1 = t(firstInd:indices(i)-10,:)-t(firstInd,:); %<-------------10 is hard coded here. This means 10 data points for single support 1
    
    u1dot1 = [(-1/2).*exp(1).^((-1).*t_temp1).*((1+exp(1).^(2.*t_temp1)).*gamma+(-1).*(1+exp(1).^( ...
   2.*t_temp1)).*teta0_temp+tetad0_temp+(-1).*exp(1).^(2.*t_temp1).*tetad0_temp)]; %

    tetad0_temp2 = tetad0(indices(i)-10+1);
    teta0_temp2 = teta0(indices(i)-10+1);
    t_temp2 = t((indices(i)-10+1):indices(i),:)-t(indices(i)-10+1);  %<-------------10 is hard coded here. This means 10 data points for single support 2
    
    u1dot2 = [(-1/2).*exp(1).^((-1).*t_temp2).*((1+exp(1).^(2.*t_temp2)).*gamma+(-1).*(1+exp(1).^( ...
    2.*t_temp2)).*teta0_temp2+ tetad0_temp2+(-1).*exp(1).^(2.*t_temp2).* tetad0_temp2)]; %

    u1dot = [u1dot;  u1dot1; u1dot2];

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

%point mass model
grt = 0;
cop = 0;