function [xnew,energies] = s2stransition(x, walk, parms);
% [xnew, energies] = s2stransition(x, walk) returns the new state given the state 
% prior to heel contact and a toe-off impulse P.  The effect of the
% toe-off impulse and the heel impact are computed as perfectly inelastic 
% impulses, and impulse-momentum and conservation of angular momentum
% principles are used to find the velocities after impact. There is no 
% change in configuration except that after heel strike the legs are 
% switched.  For heel strike, a component of angular momentum conserved 
% about the point of contact, and for the trailing leg about the hip.
% walk is a walk2 object containing parameters.
% This is for the 2-D rimless wheel
% x = [qstance ustance]'

% Arthur D. Kuo, see:
% Kuo, A. D. (2002) Energetics of actively powered locomotion using the 
%   simplest walking model, Journal of Biomechanical Engineering, 124: 113-120. 
% Kuo, A. D. (2001) A simple model predicts the step length-speed 
%   relationship in human walking, Journal of Biomechanical Engineering, 
%   123: 264-269. 
% McGeer (1990) Passive dynamic walking, Intl. J. Robotics Research,
%   9: 62-82.

if nargin < 3 || isempty(parms)
  parms = get(walk, 'parms');
end

rgyr = parms.rgyr; alpha = parms.alpha;
P = parms.P; % the toe-off impulse

q1 = x(1); u1 = x(2);

angle = q1; % push-off impulse is along leg
Ix = P*sin(-angle); Iy = P*cos(angle);

q1 = x(1); u1 = x(2);

c2t = cos(2*q1); s2t = sin(2*q1);

% angular momentum before: H- = I*omega + r x mv + r x P
%                             = m*rgyr^2*u1 + m*cos(2theta)*u1 + sin(2theta)*P
% angular momentum after:  H+ = I*omega + r x mv
%                             = m*rgyr^2*u1 + m*1*u1
% yielding u1(+) = (u1(-)*(cos(2theta)+rgyr^2) + sin(2theta)*P) / (1+rgyr^2)

xnew = [ -q1 ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ];

energies.before = energy(x, walk); % energies before impulse
energies.intermediate = energies.before; energies.intermediate.KE = energies.intermediate.KE + 0.5*P*P; 
energies.intermediate.total = energies.intermediate.total + 0.5*P*P;
energies.after = energy(xnew, walk);
energies.pushoffwork = 0.5*P*P;
energies.heelstrikework = energies.after.KE - energies.intermediate.KE;
