function xdot = fwalk(t,x,walk, parms, bump)
% FWALK   Right-hand side for Matlab ode45, 2-D rimlesss wheel
% xdot = fwalk(t, x, walk) computes the right-hand-side of the
% differential equations of motion, given time t and state x.
% walk is a walk object containing the parameters
% The equations of motion are actually for a nonlinear rimless
% wheel, without using a linear approximation of a pendulum

% McGeer (1990) Passive dynamic walking, Intl. J. Robotics Research,
%   9: 62-82.
% Although unlike McGeer, we define q1 as ccw from vertical

if nargin < 4 || isempty(parms)
  parms = get(walk, 'parms');
end

gamma = parms.gamma; rgyr = parms.rgyr;

q1 = x(1);
u1 = x(2);

sigma2 = 1/(1 + rgyr*rgyr); % this is sigma-squared from McGeer (1990)

theta = x(1);
thetadot = x(2);

%u1dot = sigma2*q1 - sigma2*gamma; % linear version
u1dot = sigma2*sin(q1 - gamma);

xdot = [u1; u1dot];
