function [vxs,vys] = hodograph(w, xs, energies, indices, vinterms, bumps, varargin)

if nargin == 1, % only a walk object given, so do a simulation
  [xc,tc,xs,ts,energies,indices,vinterms] = onestep(w);
end

vxs = zeros(length(ts)+length(indices),1);
vys = vxs;

velindices = zeros(length(ts)+length(indices),1);

i1 = 1;
for j = 1:length(indices)
  i2 = indices(j);
  out1 = i1 + (j-1);
  out2 = i2 + (j-1);
  velindices(out1:out2) = i1:i2;
  vxs(out1:out2) = -xs(i1:i2,2).*cos(xs(i1:i2,1));
  vys(out1:out2) = -xs(i1:i2,2).*sin(xs(i1:i2,1));
  vxs(out2+1) = vinterms(j,1);
  vys(out2+1) = vinterms(j,2);
  i1 = i2+1;
end

if nargout == 0 % no output, so plot it
  plot(vxs,vys); axis equal;
  xlabel('Fwd velocity'); ylabel('Vertical velocity'); 
end