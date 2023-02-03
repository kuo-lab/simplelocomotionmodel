function [grf,grt,cop] = groundreactionforcesOsman(t, x, walk)

%the original groundreactionforces doesn't seem to work added this looking
%at sw2

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

q1 = x(1);
u1 = x(2);

sigma2 = 1/(1 + rgyr*rgyr); % this is sigma-squared from McGeer (1990)

% u1dot = fwalk(t, x, walk); % need state-derivative
% 
% % acceleration tangential to com path
% atang = -u1dot; % minus sign is because path is cw
% 
% % acceleration normal to path, along leg
% anorm = u1*u1; % this is towards ground
% 
% grf = [c1 s1; s1 -c1]*[atang; anorm]


u1dotAll = [];
for i = 1:length(t)
    q1 = x(i,1); u1 = x(i,2); %u1 = x(i,3); u2 = x(i,4);
    c1 = cos(q1); s1 = sin(q1);

    xdot = fwalk(t(i), x(i,:), walk, parms); % need state-derivative
    u1dot = xdot(2); 
    
    u1dotAll = [u1dotAll u1dot];

    grf(i,1:2) = [ -c1*u1dot+s1*u1*u1 -s1*u1dot-c1*u1*u1+1 ];

    cop(i) = 0; 

    grt(i) = 0; % ground reaction torque about z axis
end

figure; plot(u1dotAll); hold on; plot(x(:,1), 'r');; plot(x(:,2), 'b');

grt = 0;
cop = 0;