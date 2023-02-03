function [xnew,lambdas] = heelstrikepwJc(w, xminus)

%deneme heel strike, bu gercek bir fonsikyon degil sonra sil

% calculates the new state following foot contact.
% Angular momentum is conserved about the impact point for the
% whole machine, and about the hip joint for the trailing leg.
% After conservation of angular momentum is applied, the legs
% are switched.
% State vector: qstance, qswing, qdotstance, qdotswing
%
% This version uses the constraint Jacobian, and allows for an
% optional second output, lambdas

% by Art Kuo for ME 646

% Expects the following globals: L gamma Il C M alpha R Mp Ip g

 L = 1
 gamma = 0 
 Il = 0 
 C = 0 
 M = 1 
 R = 0 
 Mp = 1 
 Ip = 0 


sg = sin(gamma);
cg = cos(gamma);

q1 = xminus(1); q2 = xminus(2); % q1 is angle of stance leg ccw wrt vertical
u1 = xminus(3); u2 = xminus(4); % q2 is angle of swing leg ccw wrt vertical

q2 = -q1;

c1 = cos(q1); c2 = cos(q2); c12 = cos(q1-q2);
s1 = sin(q1); s2 = sin(q2); s12 = sin(q1-q2);

% The constraint Jacobian will be used to perform the augmented
% Newton-Euler method, in terms of the "maximal" coordinates X.
% But since our simulation already uses the minimal state vector x,
% we first need to convert from x to X:

% Calculate pose Jacobian, giving center of mass motions of each segment
% so that Jp*u yields something like this: 
%   [x1dot; y1dot; th1dot; xpdot; ypdot; thpdot; x2dot; y2dot;
%   th2dot]
% referring to accelerations in x, y, and theta for segments 1, 2 and
% the pelvis p (which we're treating as a separate segment)

Jp = [-(R+(C-R)*c1)        0      ;  % velocity of stance leg COM x
        -(C-R)*s1          0      ;  % velocity of stance leg COM y
              1            0      ;  % angular velocity of stance leg
     -(R+(L-R)*c1)         0      ;  % velocity of pelvis x
        -(L-R)*s1          0      ;  % velocity of pelvis y
              1            0      ;  % angular velocity of pelvis
     -(R+(L-R)*c1)      (L-C)*c2  ;  % velocity of swing leg COM x
        -(L-R)*s1       (L-C)*s2  ;  % velocity of swing leg COM y
              0            1      ]; % angular velocity of swing leg          

% Let Xdot be the vector containing the full velocities of all segments:
% stance leg, pelvis, and swing leg.
Xdotold = Jp * [u1;u2];

% Now perform calculations for Xdotnew, using a big mass matrix:

bigM = diag([M M Il Mp Mp Ip M M Il]); % diagonal matrix of segment masses
% Notice that bigM * Xdotold is the momentum of the system before impact

% Let's also put together a matrix of constraints for what happens after
% the system hits the ground. We want the segments still to be stuck
% together, and we want the leading foot to be stuck to the
% ground after an inelastic impact. Set up a constraint Jacobian so that the
% constraints are satisfied with Jc * Xdot = 0
% where Jc treats all degrees of freedom in terms of before impact
Jc   = [1 0 -(L-C)*c1 -1  0  0 0 0       0   ;  % trailing leg glued to pelvis
        0 1 -(L-C)*s1  0 -1  0 0 0       0   ;  % 
        0 0      1     0  0 -1 0 0       0   ;  % pelvis rotates with leg
        0 0      0    -1  0  0 1 0  -(L-C)*c2;  % leading leg glued to pelvis
        0 0      0     0 -1  0 0 1  -(L-C)*s2;
        0 0      0     0  0  0 1 0 R+(C-R)*c2;  % leading foot x glued to ground
        0 0      0     0  0  0 0 1 (C-R)*s2  ]; % leading foot y glued to ground

% Note that gravity produces no impulse during the collision
rhs = [bigM*Xdotold; zeros(7,1)];

blockmatrix = [bigM Jc'; Jc zeros(7,7)];

blocklhs = blockmatrix \ rhs; % solve for the new velocities

% blocklhs contains the new Xdots, plus the constraint forces

Xdotnew = blocklhs(1:9);
lambdas =  blocklhs(10:end); % these are internal forces lambda

% The Xdots of most interest are the angular velocities of 
% the trailing and leading legs
utrail = Xdotnew(3);
ulead  = Xdotnew(9);

unew = [ulead; utrail];
xnew = [xminus(2); xminus(1); unew(1); unew(2)];
% be sure test whether this produces the correct output
