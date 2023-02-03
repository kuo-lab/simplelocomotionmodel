
function [xe,te,x,t,energies,indices,vinterms] = doTheSwitchNinterateUntilHeelStrike(w, x0, ctrls, latePushOffFlag, varargin);

%this begins right before HS and hip torque is added

anim = 0; AbsTol = 1e-8; RelTol = 1e-7; tf = 5.5; %<--------------ORIGINAL

opt_argin = varargin;
while length(opt_argin) >= 2,
  opt = opt_argin{1};
  val = opt_argin{2};
  opt_argin = opt_argin(3:end);
  switch opt
    case 'anim'
      anim = val;
    case 'AbsTol'
      AbsTol = val;
    case 'RelTol'
      RelTol = val;
    case 'tf'
      tf = val; % here tf is defined as the time beyond the start of this step
    otherwise
      error('Onestep options: anim, AbsTol, RelTol, tf')
  end
end

% Integrate the ODE, using m-files or mex-files

% To do this with m-files and without using mex files, use
%options = odeset('events',@heelstrikeevent,'AbsTol',AbsTol,'RelTol',RelTol);
parms = get(w, 'parms'); gamma = parms.gamma; alpha = parms.alpha;
bumps = get(w,'bumps');
% x0(1) = -x0(1);
options = odeset('AbsTol',AbsTol,'RelTol',RelTol);
t = []; x = []; t0 = 0; indices = []; vinterms = [];  
for i = 1:size(ctrls, 2)%length(bumps)
    
  bump = bumps(i);
  ctrl = ctrls(:,i);
  
  T = ctrl(2);
%   x0 = [-0.3 ctrl(3)];
  tf = ctrl(4);

    if latePushOffFlag   
        bumpIndexes = find(bumps ~= 0); 
        if  bumpIndexes
            if  (i == bumpIndexes(1)) || (i == bumpIndexes(2))
                 w = set(w,'latePushOffFlag',1);
            else
                 w = set(w,'latePushOffFlag',0);
            end
        else
            w = set(w,'latePushOffFlag',1);
        end
    end

  % begin with the switch: 
  [xe_temp, energyinfo, vinterm] = s2stransition(x0, w, parms, bump, ctrl(1));  %ctrl(1)%<---------------------------------

  w = set(w,'T', T);
  sol = ode45(@fwalk2,[t0 t0 + tf], xe_temp(:), options, w, parms, bump);
 
  t = [t; sol.x']; x = [x; sol.y']; indices = [indices; length(t)];
  % where indices gives time points of the heelstrike times
  vinterms = [vinterms; vinterm];
 
  x0 =  x(end,:); 
  t0 = t(end);
  
  energyinfo.hipwork = T*(x(end,1)-x(1,1)); %<--------------------- 
  energies(i) = energyinfo; 
end

te = t(end); 
xe = x(end,:); %<---------------------------------


if nargout == 0
   plot(t,[x(:,1) x(:,2)]); % plot stance and swing angles
   xlabel('time'); ylabel('states');
end	

if ~(all(isreal(t)) | all(all(isreal(x))))
  xe = [NaN NaN];
  te = NaN;
  
end %if


end % function



%if it begins on a bump %<---------------------
% if bumps(1) ~= 0 
%     firstStepOnBump = 1;
% else
%     firstStepOnBump = 0;
% end



% this step can be called in the main script
%   if firstStepOnBump == 1%if it begins on bump
%       w_temp = walkrw2unevenl();
%       w_temp = set(w_temp,'bumps', [0 bumps(1)]); %remember to shift the bump here to get the last state before the bump
%       [xc_temp ,tc_temp ,xs_temp ,ts_temp ,energies_temp ,indices_temp ] = onestep2(w_temp,[],[],[],0);
%       x0 = xs_temp(indices_temp(1),:);   %becareful the first index is the on bump state
%       clear xc_temp tc_temp xs_temp ts_temp energies_temp indices_temp
%       firstStepOnBump = 0;
%   end
