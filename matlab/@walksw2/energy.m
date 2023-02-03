function [varargout] = energy(walk,x,t)
% ENERGY  returns total energy of simple 2-D walker 
% E = energy(walk,x) takes in the state vector and
% returns the total energy, kinetic energy, and potential
% energy of the walker for that state vector.
% E is a structure with the following variables:
%   E.total = total kinetic & potential energy
%   E.KE    = total kinetic only
%   E.PE    = total potential energy
%   E.PEg   = total gravitational potential energy
%   E.PEs   = total spring potential energy
% E = energy(walk) uses the fixed point 'xstar' for x.
% [KE, PE, PEg, PEs] = energy(w) returns the same information
%   as four scalars, if the user does not want a structure

if nargin == 0
  error('energy: need a walk object as first argument');
elseif nargin == 1 % optional arguments
  if ~isa(walk, 'walksw2')
    error('energy: need a walk object as first argument');
  end
  [xe,te,x,t] = onestep(walk);
elseif nargin == 2
  if length(x) == 4   % and it's a vector, meaning an initial condition
    t = 0;
  elseif size(x,1) > 1 && size(x,2) == 4 % it's a matrix of states
    t = linspace(0,100,size(x,1)); % Time as a percentage of the length of the series of states
  else
    error('energy: unknown second argument')
  end
elseif nargin > 3
  error('incorrect number of arguments');
end

parms = get(walk, 'parms');
gamma = parms.gamma; Kp = parms.Kp; 

for i = 1:length(t)
    q1 = x(i,1); q2 = x(i,2); u1 = x(i,3); u2 = x(i,4);

    KE(i) = 0.5*u1^2;
    PEg(i) = cos(gamma-q1);
    PEs(i) = 0; %0.5*Kp*q2^2;
    PE(i) = PEg(i) + PEs(i);
end

E.total = KE + PE;
E.KE = KE;
E.PE = PE;
E.PEg = PEg;
E.PEs = PEs;

varargout{1} = E;

if nargout == 0 && length(t) > 1
    plot(t,KE,t,PE-1,t,PEg-1,t,PEs); legend('KE','PE','PEg','PEs');
elseif nargout == 4 % user wants four scalar outputs instead of a structure
    varargout{1} = KE;
    varargout{2} = PE;
    varargout{3} = PEg;
    varargout{4} = PEs;
end