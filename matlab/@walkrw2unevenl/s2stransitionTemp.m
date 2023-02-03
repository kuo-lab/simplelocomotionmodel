function [xnew,energies,vinterm] = s2stransitionTemp(x, walk, parms, bump, ctrl)

% this function is to find the numerical derivative of the model and
% the main difference is it returns x(1) differently 2*alpha+x(1).
% x(1) in this configuration should be negative


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


% DO NOT USE HIP IMPULSE
% Hroot2work = 0; % by default no hip work
% if nargin >= 5 && ~isempty(ctrl) % a control input is given
%   P = ctrl(1);
%   if length(ctrl) == 2  % handle a hip impulse
%     Hroot2work = ctrl(2);
%   end
% end
P = ctrl(1);  %<-----------------------------
q1 = x(1); u1 = x(2);

angle = q1; % push-off impulse is along leg
Ix = P*sin(-angle); Iy = P*cos(angle);

q1 = x(1); u1 = x(2);
% c2t = cos(2*q1); s2t = sin(2*q1);  %ORIGINAL
c2t = cos(-2*alpha); s2t = sin(-2*alpha);  %<--------------

% angular momentum before: H- = I*omega + r x mv + r x P
%                             = m*rgyr^2*u1 + m*cos(2theta)*u1 + sin(2theta)*P
% angular momentum after:  H+ = I*omega + r x mv
%                             = m*rgyr^2*u1 + m*1*u1
% yielding u1(+) = (u1(-)*(cos(2theta)+rgyr^2) + sin(2theta)*P) / (1+rgyr^2)

if get(walk,'latePushOffFlag')
    xnew = [ alpha+bump  -sqrt(P^2+(c2t*u1)^2)];      % RGYR IS ASSUMED TO BE ZERO
    vinterm = [ cos(q1)*(c2t*-u1) sin(q1)*(c2t*-u1)]; %vinterm is the velocity after heel strike before push off because push off is late
    xinterm = [ q1  -c2t*u1];  %
else
    xnew = [ 2*alpha+x(1) ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ];
    vinterm = [ cos(q1)*(-u1)+P*sin(-q1) sin(q1)*(-u1)+P*cos(-q1)];  %vinterm is the velocity after push off before heel strike
end
% xnew = [ alpha+bump ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ]; %ORIGINAL
% intermediate velocity in x and y components
% vinterm = [ cos(q1)*(-u1)+P*sin(-q1) sin(q1)*(-u1)+P*cos(-q1)]; %ORIGINAL

energies.before = energy(x, walk);   % energies before impulse
energies.after = energy(xnew, walk); % energies after impulse
% intermediate state's energy and heel strike work depends whether there the push off is late
if get(walk,'latePushOffFlag')
    energies.intermediate = energy(xinterm, walk); 
    energies.heelstrikework = energies.intermediate.KE - energies.before.KE;
    energies.pushoffwork = 0.5 * (xnew(2)^2 - xinterm(2)^2);
else
    energies.intermediate = energies.before; energies.intermediate.KE = energies.intermediate.KE + 0.5*P*P; 
    energies.intermediate.total = energies.intermediate.total + 0.5*P*P;
    energies.heelstrikework = energies.after.KE - energies.intermediate.KE;
    energies.pushoffwork = 0.5*P*P; 
end








% add a hip impulse DO NOT
% xnew(2) = -sqrt(xnew(2)^2 + sign(Hroot2work)*Hroot2work^2);
% energies.hipwork = 0.5*Hroot2work^2;
% energies.aftercollision = energies.after;
% energies.after = energy(xnew, walk);