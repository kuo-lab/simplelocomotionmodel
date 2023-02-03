function displayfields(w)
% WALK2/DISPLAYFIELDS Command window display of fields of a walkrw2
name = w.parent;
if ~isempty(name) % display the parent if there is one
  displayfields(w.(name))
end

fprintf(1,'  bumps = ['); fprintf(1, ' %f', w.parms.bumps); %w.parms.bumps(1:min(end,10))
fprintf(1,' ]\n');
fprintf(1,'  controls = ['); fprintf(1, ' %f', w.parms.controls); % w.parms.controls(1:min(end,10))
fprintf(1,' ]\n');
fprintf(1,'  latePushOffFlag  = '); fprintf(1, ' %f', w.parms.latePushOffFlag);
fprintf(1,' \n');
fprintf(1,'  T  = '); fprintf(1, ' %f', w.parms.T);
fprintf(1,' \n');
% only display at most 10 elements from each vector