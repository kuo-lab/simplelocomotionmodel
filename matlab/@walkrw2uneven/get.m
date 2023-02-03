function val = get(w,prop_name)
% GET Get asset properties from the specified object
% and return the value

% This function is designed to be used with inherited classes
% by copying it to the new class and making minimal or no modifications
% in the indicated section.

objparmnames = fieldnames(w.parms);
if any(strcmp(prop_name, objparmnames))
  val = w.parms.(prop_name);
else
  switch prop_name
    case 'parms'
      if ~isempty(w.parent)
        parentparms = get(w.(w.parent),'parms');
        pfieldnames = fieldnames(parentparms);
        pvalues = struct2cell(parentparms);
        wparms = w.parms;
        wfieldnames = fieldnames(wparms);
        wvalues = struct2cell(wparms);
        val = cell2struct( cat(1,pvalues,wvalues), cat(1,pfieldnames,wfieldnames), 1);
      else
        val = w.parms;
      end
    case 'parent'
      val = w.parent;
    otherwise
      try
        val = get(w.(w.parent), prop_name);
      catch
        error([prop_name,' Is not a valid asset property'])
      end
  end
end