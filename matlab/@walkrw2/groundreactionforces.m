function [grf,grt,cop] = groundreactionforces(t, x, walk)
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

% u1dot = fwalk(t, x, walk); % need state-derivative %ORIGINAL
u_dot = [];
for i = 1:length(x)
    u_dot(i,:) = fwalk2(t(i), x(i,:), walk, parms, get(walk, 'bumps')); % %<--------osman
end
% find(u1dot(:,1)-x(:,2) ~=0) %this sould be empty

u1dot= u_dot(:,2);

% acceleration tangential to com path
atang = -u1dot; % minus sign is because path is cw


u1 = x(:,2);
% acceleration normal to path, along leg
anorm = u1.*u1; % this is towards ground

c1 = cos(x(:,1)); s1 = sin(x(:,1));%<--------osman, this was missing


% grf = [c1 s1; s1 -c1]*[atang; anorm]%orginal

grf = [c1.*atang + s1.*anorm  s1.*atang-c1.*anorm+1];

grt = 0;
cop = 0;