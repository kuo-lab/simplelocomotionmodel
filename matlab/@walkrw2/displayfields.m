function displayfields(w)
% WALK2/DISPLAYFIELDS Command window display of fields of a walkrw2
name = w.parent;
if ~isempty(name) % display the parent if there is one
  displayfields(w.(name))
end

fprintf(1,'  rgyr = %g, alpha = %g \n', ...
      w.parms.rgyr, w.parms.alpha);
