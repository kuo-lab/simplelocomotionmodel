function display(wobject)
% WALKSW2/DISPLAY Command window display of a walksw2

% This function is designed so that it should work with inherited objects
% simply by copying it to the new object's directory.  The function
% displayfields will need to be modified.
linelength = 70;

 for i = 1:length(wobject(:))
   w = wobject(i); 
   disp(' ');
   fprintf(1,[inputname(1),' = ' class(wobject) '\n']) % display object's class
%   % then display the fixed point  
%   fprintf(1,'  xstar = [ '); fprintf(1, '%g ', get(w, 'xstar')); fprintf(1, ']\n');
% 
%   % then display all the parameters in formatted lines
%   parms = get(wobject, 'parms');
%   parmnames = fieldnames(parms);
%   longline = [];
%   for j = 1:length(parmnames)
%     longline = [longline sprintf('  %s = %g', parmnames{j}, getfield(parms, parmnames{j}))];
%   end
%   % put in linefeeds to keep lines under specified linelength
%   rem = longline;
%   while length(rem) > linelength
%     spaces = strfind( rem(1:linelength), '  ');
%     cutoff = spaces(end)-1;
%     fprintf('%s\n', rem(1:cutoff));
%     rem = rem(cutoff+1:end);
%   end
%   fprintf('%s\n\n', rem);
    displayfields(wobject); % if you want full control over the format of the parameters,
  %                      use displayfields instead of all the formatting
  %                      above, but you'll have to provide displayfields
end