function [xnew,energies,vinterm] = s2stransition(x, walk, parms, bump, ctrl)
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

rgyr = parms.rgyr; 

% alpha = parms.alpha;%this always gets the default alpha whihc is fine if alpha is constant
alpha = get(walk, 'alpha');


% P = parms.P; % the toe-off impulse

P = ctrl(1);  %<-----------------------------

%you can add Hip impulse read below line 84
% if length(ctrl) == 2 
%     Hroot2work = ctrl(2);
% end

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
%     vinterm = [ cos(q1)*(c2t*-u1) sin(-q1)*(c2t*-u1)]; %vinterm is the velocity after heel strike before push off because push off is late
    temp = alpha+bump;
    vinterm = [ cos(temp)*(c2t*-u1) sin(temp)*(c2t*-u1)];
    xinterm = [ temp  -c2t*u1];  %
else
    xnew = [ alpha+bump ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ]; %ORIGINAL
    vinterm = [ cos(q1)*(-u1)+P*sin(-q1) sin(q1)*(-u1)+P*cos(-q1)];  %vinterm is the velocity after push off before heel strike %ORIGINAL

%     %delete later************************
      %constraint po if the angle is not alpha
%     if abs(q1) < alpha        
%         P = P * (-q1)/alpha;
%     end
%     xnew = [ alpha+bump ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ]; %ORIGINAL
%     vinterm = [ cos(q1)*(-u1)+P*sin(-q1) sin(q1)*(-u1)+P*cos(-q1)];  %vinterm is the velocity after push off before heel strike %ORIGINAL
%     
%     if abs(q1) < alpha 
%       %add hip imp. 
%         HipImp = ctrl(2)*abs(q1+alpha)/alpha;
%         xnew(2) = -sqrt(xnew(2)^2 + HipImp^2);
%         energies.hipwork = 0.5*HipImp^2;
%     end
%     %delete later************************  
end
% xnew = [ alpha+bump ((c2t + rgyr*rgyr)*u1 + s2t*P)/(1 + rgyr*rgyr) ]; %ORIGINAL
% intermediate velocity in x and y components
% vinterm = [ cos(q1)*(-u1)+P*sin(-q1) sin(q1)*(-u1)+P*cos(-q1)]; %ORIGINAL


energies.hipwork = 0; 
%if you want it you can add HipImp now BUT then you need to define the
%intermediate term and energy change the energy calculations below and
%vinterm above
% xnew2_old = xnew(2);
% xnew(2) = xnew(2) + HipImp;
% energies.hipwork =  0.5 * (xnew(2)^2 - xnew2_old^2);



energies.before = energy(x, walk);   % energies before impulse
energies.after = energy(xnew, walk); % energies after impulse
% intermediate state's energy and heel strike work depends whether the push off is late
if get(walk,'latePushOffFlag')
    energies.intermediate = energy(xinterm, walk); 
    energies.heelstrikework = energies.intermediate.KE - energies.before.KE;
    energies.pushoffwork = 0.5 * (xnew(2)^2 - xinterm(2)^2);
else
    energies.intermediate = energies.before; energies.intermediate.KE = energies.intermediate.KE + 0.5*P*P; 
%     energies.intermediate.total = energies.intermediate.total + 0.5*P*P; %ORIGINAL
    energies.heelstrikework = energies.after.KE - energies.intermediate.KE;
    energies.pushoffwork = 0.5*P*P; %ORIGINAL  
    energies.intermediate.total = energies.intermediate.total + 0.5*P*P + energies.hipwork;
end




% ART HIP IMPULSE
% Hroot2work = 0; % by default no hip work
% if nargin >= 5 && ~isempty(ctrl) % a control input is given
%   P = ctrl(1);
%   if length(ctrl) == 2  % handle a hip impulse
%     Hroot2work = ctrl(2);
%   end
% end

% how Art does it: look at the sign and also he sends the sqrt root of hip impulse work as a parameter 
% xnew(2) = -sqrt(xnew(2)^2 + sign(Hroot2work)*Hroot2work^2);
% energies.hipwork = 0.5*Hroot2work^2;
% energies.aftercollision = energies.after;
% energies.after = energy(xnew, walk);




