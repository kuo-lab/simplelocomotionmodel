function xnew = s2transitionSW2deneme(w, x, P)

% push-off function applied

M = 1;
m = 1; % 

% State assignments
q1 = x(1); q2 = x(2);  qdot = x([3:4])';
% X = [xp yp xk yk xtf ytf xlf ylf qp qk qsh]

% q2 = q2-q1;
q2 = q1-q2;
%usem Jp2 from mathmatica as Jp here
% Jp = zeros(8,2);
% Jp(1,1) = -cos(q1); 
% Jp(2,1) = -sin(q1); 
% Jp(3,1) = -cos(q1); Jp(3,2) = cos(q2); 
% Jp(4,1) = -sin(q1); Jp(4,2) = sin(q2); 
% Jp(7,1) = 1; 
% Jp(8,2) = 1;  

Jp = zeros(8,2);
Jp(1,1) = -cos(q1); 
Jp(2,1) = -sin(q1); 
Jp(3,1) = -cos(q1); Jp(3,2) = cos(q2); 
Jp(4,1) = -sin(q1); Jp(4,2) = sin(q2); 
Jp(7,1) = 1; 
Jp(8,2) = 1; 


len = size(Jp,1);

Xdotold = Jp*qdot;
MM = diag([M, M, m, m, m, m, 0, 0]);


% MM = diag([M, M, m, m, 0, 0, 0, 0]);

Fncimpulse = zeros(len,1);

Pimpulse = zeros(len,1);
% Pimpulse(3) = -P*sin(q1); Pimpulse(4) = P*cos(q1);% push-off acts on tf
Pimpulse(5) = -P*sin(q1); Pimpulse(6) = P*cos(q1);% push-off acts on lf
% X = [xp yp xtf ytf xlf ylf qp qk ]

%push off constraints
% JcP = zeros(4,8);
% JcP(1,1) = -1; JcP(1,3) = 1; JcP(1,8) = -cos(q2); 
% JcP(2,2) = -1; JcP(2,4) = 1; JcP(2,8) = -sin(q2); 
% JcP(3,1) = -1; JcP(3,5) = 1; JcP(3,7) = -cos(q1); 
% JcP(4,2) = -1; JcP(4,6) = 1; JcP(4,7) = -sin(q1); 

JcP = zeros(4,8);
JcP(1,1) = -1; JcP(1,3) = 1; JcP(1,8) = -cos(q2); 
JcP(2,2) = -1; JcP(2,4) = 1; JcP(2,8) = -sin(q2); 
JcP(3,1) = 1; JcP(3,5) = -1; JcP(3,7) = cos(q1); 
JcP(4,2) = 1; JcP(4,6) = -1; JcP(4,7) = sin(q1); 




len2 = size(JcP,1);
% X = [xp yp xtf ytf xlf ylf qp qk ]
bigmatrix = [MM JcP'; JcP zeros(len2,len2)];
righthandside = [Pimpulse+MM*Xdotold; zeros(len2,1)];
soln = bigmatrix \ righthandside;
Xdotpostpush = soln(1:len,1)
lambdaimpulse = soln(len+1:end,1)

%apply heel strike using post push off vels
%heelstrike constraints
% JcH = zeros(6,8);
% JcH(1,1) = -1; JcH(1,3) = 1; JcH(1,8) = -cos(q2); 
% JcH(2,2) = -1; JcH(2,4) = 1; JcH(2,8) = -sin(q2); 
% JcH(3,1) = -1; JcH(3,5) = 1; JcH(3,7) = -cos(q1); 
% JcH(4,2) = -1; JcH(4,6) = 1; JcH(4,7) = -sin(q1); 
% JcH(5,5) = 1; 
% JcH(6,6) = 1; 
% X = [xp yp xtf ytf xlf ylf qp qk ]
% this is leading leg touching
% JcH = zeros(6,8);
% JcH(1,1) = -1; JcH(1,3) = 1; JcH(1,8) = -cos(q2); 
% JcH(2,2) = -1; JcH(2,4) = 1; JcH(2,8) = -sin(q2); 
% JcH(3,1) = 1; JcH(3,5) = -1; JcH(3,7) = cos(q1); 
% JcH(4,2) = 1; JcH(4,6) = -1; JcH(4,7) = sin(q1);  
% JcH(5,5) = 1; 
% JcH(6,6) = 1; 

% this is trailing leg touching
JcHorg = zeros(6,8);
JcHorg(1,1) = -1; JcHorg(1,3) = 1; JcHorg(1,8) = -cos(q2); 
JcHorg(2,2) = -1; JcHorg(2,4) = 1; JcHorg(2,8) = -sin(q2); 
JcHorg(3,1) = 1; JcHorg(3,5) = -1; JcHorg(3,7) = cos(q1); 
JcHorg(4,2) = 1; JcHorg(4,6) = -1; JcHorg(4,7) = sin(q1); 
JcHorg(5,3) = 1; 
JcHorg(6,4) = 1; 
JcHorgT = JcHorg';


% this is trailing leg only Y touching
% JcH = zeros(5,8);
% JcH(1,1) = -1; JcH(1,3) = 1; JcH(1,8) = -cos(q2); 
% JcH(2,2) = -1; JcH(2,4) = 1; JcH(2,8) = -sin(q2); 
% JcH(3,1) = 1; JcH(3,5) = -1; JcH(3,7) = cos(q1); 
% JcH(4,2) = 1; JcH(4,6) = -1; JcH(4,7) = sin(q1); 
% JcH(5,4) = 1; 

% this is trailing leg touching and pelvis connected to trailing leg no
% lead leg
JcH = zeros(4,6);
JcH(1,1) = -1; JcH(1,3) = 1; JcH(1,6) = -cos(q2); 
JcH(2,2) = -1; JcH(2,4) = 1; JcH(2,6) = -sin(q2); 
JcH(3,3) = 1; 
JcH(4,4) = 1; 

%eliminate q2
JcH(:,5) = []; 

% m = 0
% MM = diag([M, M, m, m, m, m, 0, 0]);

% calisan stance*****************
m = 1
MM = diag([M, M, m, m, 0]);
len = size(MM,1);
XdotpostpushOrg = Xdotpostpush;
Xdotpostpush([5 6 8]) = []; 
%*************

%sil
% MM = diag([M, M, m, m, 0]);
% XdotpostpushOrg = Xdotpostpush;
% Xdotpostpush([1 2 5 6 7]) = []; 
% 
% Xdotpostpush = [Xdotpostpush(1:2);Xdotpostpush]
%***

len3 = size(JcH,1);
% gravity produces no impulse during the collision
rhs = [MM*Xdotpostpush; zeros(len3,1)];
blockmatrix = [MM JcH'; JcH zeros(len3,len3)];
blocklhs = blockmatrix \ rhs; % solve for the new velocities

Xdotnew = blocklhs(1:len);
lambdas =  blocklhs(len+1:end); % these are internal forces lambda

xdot = Xdotnew(end-1:end);
xnew = [ x(1) x(2)  xdot'];




% *****************
JcH4 = zeros(4,6);
JcH4(1,1) = 1; JcH4(1,3) = -1; JcH4(1,5) = cos(q1); JcH4(1,6) = -cos(q2); 
JcH4(2,2) = 1; JcH4(2,4) = -1; JcH4(2,5) = sin(q1); JcH4(2,6) = -sin(q2); 
JcH4(3,1) = 1; 
JcH4(4,2) = 1;





% X = [xp yp xtf ytf xlf ylf qp qk ]
m = 1
MM = diag([0, 0, m, m, m, m, 0, 0]);
% MM = diag([m, m, m, m, 0, 0]);
len = size(MM,1);

Xdotpostpush = XdotpostpushOrg;
% Xdotpostpush([1 2]) = []; 
%*************
% JcH(:,1:2) = [];
% JcH(3:4,:) = [];
% JcH = eye(2);
JcH = JcHorg;%(3:4,5:end-1);
% JcH = JcH4;
len3 = size(JcH,1);
% gravity produces no impulse during the collision
rhs = [MM*Xdotpostpush; zeros(len3,1)];
blockmatrix = [MM JcH'; JcH zeros(len3,len3)];
blocklhs = blockmatrix \ rhs; % solve for the new velocities

Xdotnew2 = blocklhs(1:len);
lambdas2 =  blocklhs(len+1:end); % these are internal forces lambda