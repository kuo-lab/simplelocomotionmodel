function E = energy2(varargin);
% ENERGY2  returns total energy of 2-D rimless wheel
% E = energy2(x, walk) takes in the state vector and
% returns the total energy, kinetic energy, and potential
% energy of the walker for that state vector.
% E is a structure with the following variables:
%   E.total = total kinetic & potential energy
%   E.KE    = total kinetic only
%   E.PE    = total potential energy
%   E.PEg   = total gravitational potential energy

if nargin == 1,
  walk = varargin{1};
  x = get(walk, 'xstar');
elseif nargin == 2
  walk = varargin{2};
  x = varargin{1};
else
  error('incorrect number of arguments');
end

parms = get(walk, 'parms');
gamma = parms.gamma; rgyr = parms.rgyr;

q1 = x(1); u1 = x(2); 

cgq1 = cos(gamma-q1);

PEg = cgq1;

KE = (1 + rgyr*rgyr) * u1*u1;
        
KE = 0.5 * KE;
E.total = KE + PEg;
E.KE = KE;
E.PE = PEg;
