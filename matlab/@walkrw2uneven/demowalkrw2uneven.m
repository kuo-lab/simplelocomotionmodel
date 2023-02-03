%% Rimless wheel walking on uneven terrain
% Simultion study of walking with up-down steps, plus two kinds of
% random bumpiness: uniform and normal distributions of bumps.
% Full simulations with nonlinear pendulum ODE

%% Set up rimless wheel normally and with bumps

wrw2 = walkrw2;      % original rimless wheel
wu = walkrw2uneven();  % uneven terrain
wub = gradsearch(set(wu, 'bumps', [-0.05 0.05]),[],'info',0); % a limit cycle with bumps
wul = gradsearch(walkrw2unevenl(),[],'info',0); % uneven terrain, linearized pendulum (several times faster)
wubl = gradsearch(set(wul, 'bumps', [-0.05 0.05]),[],'info',0);

% Plot the states for the two models
clf; hold all
onestep(wu); legend('q1', 'q1dot');
onestep(wub); 
onestep(wul); onestep(wubl); hold off

wul = walkrw2unevenl();

%mycolors = flipud([254,217,118; 254,178,76; 253,141,60; 252,78,42; 227,26,28; 177,0,38]/255);
mycolors = flipud([255,245,240; 254,224,210; 252,187,161; 252,146,114; 251,106,74; 239,59,44; 203,24,29; 165,15,21; 103,0,13]/255);
% from colorbrewer2.org a list of colors with progression of levels

%% Demo a few operations
% The uneven walker can be simulated using two functions: 
% manybumps, starting on the level gnd fixed pt;
% and onestep where bumps can be incorporated into the parameters.
% Demonstrate that step times are slower on bumps

[xc,normaltime,xs,ts,normalenergies] = onestep(wrw2);

[xc,tc] = onestep(wu);
% manybumps: demonstrate several normal steps, using the automatic plot
manybumps(wu, [], [0 0 0 0 0 0]); % where the list is a bunch of bump heights
fprintf(1,'average step times normally: %f\n', tc) % each step time

clf; subplot(121)
% and now do the same for some bumps
upanddownbumps = repmat([0.01 -0.01],1,10); 
[xd,td,xs,ts] = manybumps(wu, [], upanddownbumps, 'plotstep', 1);
fprintf(1,'average step times with bumps: %f\n', mean(diff([0; td]))) % each step time

% onestep: here is another way to run several steps, by incorporating the bumps into
% the class itself. Here onestep is overloaded to provide this function,
% and when bumps is a vector, it takes as many steps as there are bumps
subplot(122)
wub2 = set(wu, 'bumps', upanddownbumps);
onestep(wub2);

wub4 = findgaitspeed1(wub, 'speed', 0.4, 'parmvary', 'P','info',0);
[xc,tc,xs,ts,energies,indices] = onestep(wub4);
speedinfo = walkspeed(wub4);
costoftransport = 0.5*(get(wub4,'P')^2)/(speedinfo.steplength)

% Hodograph for many steps
%plot(-xs(:,2).*cos(xs(:,1)),-xs(:,2).*sin(xs(:,1))); axis equal
% or alternatively,
%hodograph(wub4);

%% Step time goes up with bumpiness squared 
% For rimless wheel with up-and-down bumps plus randomly distributed.
% Show that the average step times increase quadratically with bumpiness
% and that random bumps are more costly than up-and-down bumps, mainly
% because the distributions have tails, and it is the tails that cause
% very uneven step times.
clear fupdown
fupdown.heights = linspace(0, 0.05, 21); fupdown.steptimes = 0*fupdown.heights;
fupdown.bumps = repmat([1 -1],1,10);
for i = 1:length(fupdown.heights)
  [xd,td,xs,ts] = manybumps(wu, [], fupdown.bumps*fupdown.heights(i));
  fupdown.steptimes(i) = mean(diff([0; td]));
  fupdown.steplengths(i) = 2*sin(get(wu, 'alpha'));
end
clf; plot(fupdown.heights, fupdown.steptimes)
% indeed, time goes up quadratically

% a smarter way is to just use the limit cycle for up and down
lupdown.heights = linspace(0, 0.05, 21); lupdown.steptimes = 0*lupdown.heights;
lupdown.bumps = [1 -1];
w = wu;
for i = 1:length(lupdown.heights)
  w = gradsearch(set(w,'bumps',lupdown.heights(i)*lupdown.bumps),[],'info',0);
  [xc,tc,xs,ts,energies,indices] = onestep(w);
  lupdown.steptimes(i) = mean(diff([0; ts(indices)]));
  lupdown.steptimes2(i,1:2) = ts(indices)';
  lupdown.steplengths(i) = 2*sin(get(w,'alpha'));
end
plot(lupdown.heights, lupdown.steptimes, fupdown.heights, fupdown.steptimes)
bumps = lupdown.heights*std(lupdown.bumps,1); % quadratic fit for bumps
X = [bumps'.^2 1+0*bumps'];
lupdown.pfit = regress(lupdown.steptimes', X);

% uniformly distributed bumps
rng('default')
numsteps = 100;
clear frand
frand.bumps = 2*rand(1, numsteps); frand.bumps = frand.bumps - mean(frand.bumps); % uniform -1 to 1
frand.heights = linspace(0, 0.05, 21); frand.steptimes = 0*frand.heights;
for i = 1:length(frand.heights)
  [xd,td,xs,ts,sis,es] = manybumps(wu, [], frand.bumps*frand.heights(i));
  steptimes = diff([0; td]);
  frand.steptimes(i) = mean(steptimes);
  frand.steplengths(i) = 2*sin(get(wu,'alpha'));
end
% with a plot of average step times vs bump height
plot(frand.heights*std(frand.bumps,1), frand.steptimes,...
  lupdown.heights*std(lupdown.bumps,1), lupdown.steptimes)
xlabel('heights'); ylabel('times'); legend('random','up and down')

% normally distributed bumps
rng('default')
numsteps = 100;
clear frandn
frandn.bumps = 1*randn(1, numsteps); frandn.bumps = frandn.bumps - mean(frandn.bumps); % normal dist, std 1
frandn.bumps = frandn.bumps / std(frandn.bumps,1); 
frandn.heights = linspace(0, 0.025, 21); frandn.steptimes = 0*frandn.heights;
for i = 1:length(frandn.heights)
  [xd,td,xs,ts,sis,es] = manybumps(wu, [], frandn.bumps*frandn.heights(i));
  steptimes = diff([0; td]);
  frandn.steptimes(i) = mean(steptimes);
  frandn.steplengths(i) = 2*sin(get(wu,'alpha'));

end
% with a plot of average step times vs bump height
clf; plot(frandn.heights*std(frandn.bumps,1), frandn.steptimes,...,
  frand.heights*std(frand.bumps,1), frand.steptimes,...
  lupdown.heights*std(lupdown.bumps,1), lupdown.steptimes)
hold on; plot(bumps, lupdown.pfit(1)*bumps.^2+lupdown.pfit(2), 'k');
xlabel('heights'); ylabel('ave step time'); 
legend('rand norm','rand uni','up and down', '\beta^2 fit')
title('Rimless wheel on uneven terrain')

% this is for these standard deviations
fprintf(1,'Statistical info about the bumps:\n');
fprintf(1,'standard dev of up and down = %g\n', std(lupdown.bumps,1));
fprintf(1,'standard dev of random uni  = %g\n', std(frand.bumps,1));
% which is supposed to be (b-a)/sqrt(12)
fprintf(1,'standard dev of randomnorm  = %g\n', std(frandn.bumps,1));
% of course, the long tail is costly

% relative timing costs by fitting the others
X = [(frand.heights*std(frand.bumps,1))'.^2 ones(length(frand.heights),1)];
frand.pfit = regress(frand.steptimes', X);
X = [(frandn.heights*std(frandn.bumps,1))'.^2 ones(length(frandn.heights),1)];
frandn.pfit = regress(frandn.steptimes', X);
textinfo = sprintf('Relative timing costs normal : uniform : up-down\n%4.2f : %4.2f : 1\n',...
  frandn.pfit(1)/lupdown.pfit(1), frand.pfit(1)/lupdown.pfit(1));
disp(textinfo)
text(0.01, 1.47, textinfo)  

%% average step velocities as function of bumpiness
clf; plot(frandn.heights*std(frandn.bumps,1),frandn.steplengths./ frandn.steptimes,...,
  frand.heights*std(frand.bumps,1), frand.steplengths ./ frand.steptimes,...
  lupdown.heights*std(lupdown.bumps,1), lupdown.steplengths ./ lupdown.steptimes)
xlabel('heights'); ylabel('ave step speed'); 
title('Rimless wheel on uneven terrain')
% relative speeds costs by fitting to quadratic
X = [(lupdown.heights*std(lupdown.bumps,1))'.^2 ones(length(lupdown.heights),1)];
lupdown.pvfit = regress(lupdown.steplengths' ./ lupdown.steptimes', X);
hold on; plot(bumps, lupdown.pvfit(1)*bumps.^2+lupdown.pvfit(2), 'k');
X = [(frand.heights*std(frand.bumps,1))'.^2 ones(length(frand.heights),1)];
frand.pvfit = regress(frand.steplengths' ./ frand.steptimes', X);
X = [(frandn.heights*std(frandn.bumps,1))'.^2 ones(length(frandn.heights),1)];
frandn.pvfit = regress(frandn.steplengths'./frandn.steptimes', X);
textinfo = sprintf('Relative speed costs normal : uniform : up-down\n%4.2f : %4.2f : 1\n',...
  frandn.pvfit(1)/lupdown.pvfit(1), frand.pvfit(1)/lupdown.pvfit(1));
disp(textinfo)
text(0.01, 1.47, textinfo)  

legend('rand norm','rand uni','up and down', '\beta^2 fit')

%% Hodographs for a normal distribution

clf; [xd,td,xs,ts,sis,es] = manybumps(wu, [], frandn.bumps*frandn.heights(5));
plot(-xs(:,2).*cos(xs(:,1)),-xs(:,2).*sin(xs(:,1)),'linewidth',0.5); axis equal

hold all;
w = gradsearch(set(w,'bumps',lupdown.heights(5)*lupdown.bumps),[],'info',0);
[xc,tc,xs,ts,energies,indices] = onestep(w);
plot(-xs(:,2).*cos(xs(:,1)),-xs(:,2).*sin(xs(:,1)),'r-','linewidth',2); 
xlabel('fwd vel'); ylabel('vert vel');

%% Distribution of step times skews with bumpiness
% Make a plot of the distributions of step times
% and try this for a bunch of heights, for normally and uniformly
% distributed bumps.
rng('default')
numsteps = 100;
clear distp
distp.bumps = 2*rand(1, numsteps); 
distp.bumps = distp.bumps - mean(distp.bumps); % uniform -1 to 1
distp.heights = linspace(0, 0.05, 21); 
distp.heights = distp.heights(2:end); distp.bumptimes = 0*distp.heights;
clf
for i = 1:length(distp.heights)
  [xd,td,xs,ts,sis,es] = manybumps(wu, [], distp.bumps*distp.heights(i));
  steptimes = diff([0; td]);
  distp.bumptimes(i) = mean(steptimes);
  distp.collisionworks(i) = mean([es.heelstrikework]);
  % here's the distributino of collision work
  h = histfit(-[es.heelstrikework],[],'gev'); hold on; %plot([normaltime normaltime],get(gca,'ylim'),'r:');
  distp.workx{i} = get(h(2),'xdata');
  distp.worky{i} = get(h(2),'ydata');
  
  h = histfit(steptimes,[],'gev'); hold on; plot([normaltime normaltime],get(gca,'ylim'),'r:');
  distp.timex{i} = get(h(2),'xdata');
  distp.timey{i} = get(h(2),'ydata');
  plot(mean(steptimes)*[1 1], get(gca,'ylim'),'b--');  %pause
  %plot(repmat(lupdown.steptimes2(i,:),2,1), ([1;1]*get(gca,'ylim'))'); 
  hold off;
end

% Normalized plot tries to get areas right, but looks a bit too spikey
clf; subplot(211); hold all;
plot(normaltime*[1 1],get(gca,'ylim'),'--'); 
for i = 1:3:length(distp.heights)
  curvearea = trapz(distp.timex{i},distp.timey{i});
  plot(distp.timex{i},distp.timey{i}/curvearea);    % normalized plot
  [ymin,imin] = min((distp.timex{i}-distp.bumptimes(i)).^2);
  plot(distp.timex{i}(imin),distp.timey{i}(imin)/curvearea,'r.');
end
xlabel('step times'); ylabel('distribution');
title('normalized histogram of step times')

% The non-normalized timing distribution plot
% has wrong areas but makes it easier to see what's going on
subplot(212); hold all; for i=1:3:length(distp.heights); 
  plot(distp.timex{i},distp.timey{i}); 
  [ymin,imin] = min((distp.timex{i}-distp.bumptimes(i)).^2);
  plot(distp.timex{i}(imin),distp.timey{i}(imin),'r.');
end
xlabel('step times'); ylabel('distribution');
title('non-normalized histogram of step times')

% The following verifies what we already know: (commented out)
%clf;
%plot(distp.heights, distp.bumptimes)
% obviously step times go up with square of bumpiness as we already know

%% A couple other minor findings
% Step length doesn't vary much with bumpiness, and
% a gravity powered gait functions just like the push-off gait

% Show that step length doesn't vary much with bumpiness
% and I don't think step length matters:
clf; subplot(211); plot(distp.heights, mean(2*sin(0.3)*cos(distp.bumps'*distp.heights)))
xlabel('bumpiness'); ylabel('true step length'); set(gca,'ylim',[0 0.7])
title('cosine effect of bumpiness on step lengths');

% Do it again with a gravity gait
subplot(212);
wrwg = findgait(wu,set(wu,'gamma',0.06,'P',0),'info',0);
rng('default')
numsteps = 25;
bumps = 2*rand(1, numsteps); bumps = bumps - mean(bumps); % uniform -1 to 1
heights = linspace(0, 0.1, 10); bumptimes = 0*heights;
for i = 1:length(heights)
  [xd,td,xs,ts] = manybumps(wrwg, [], bumps*heights(i));
  bumptimes(i) = mean(diff([0; td]));
end
plot(heights, bumptimes)
xlabel('bumpiness'); ylabel('average step times'); title('gravity powered gait');

%% Cost goes up with square of bumpiness
% For walking at constant speed, for up-down bumps 
% and for uniformly distributed bumpiness
% Note that uniform steps cost more than three times as much as the
% up-down steps of same standard deviation.

% using a two-step limit cycle
clear ludcv
ludcv.heights = linspace(0, 0.05, 31); ludcv.steptimes = 0*ludcv.heights;
ludcv.bumps = [1 -1]; 
% This is how to find a gait where P is adjusted to get constant speed
w = findgaitspeed1(wu, 'speed', 0.4, 'parmvary', 'P','info',0);
ludcv.steptimes(1) = normaltime; % no bumps for first element
ludcv.steplengths(1) = 2*sin(get(w,'alpha'));
ludcv.Ps(1) = get(w,'P');
for i = 2:length(ludcv.heights)
  w = gradsearch(set(w,'bumps',ludcv.heights(i)*ludcv.bumps),[],'info',0);
  w = findgaitspeed1(w, 'speed', 0.4, 'parmvary', 'P','info',0,'fgscriterion',1e-4);
  [xc,tc,xs,ts,energies,indices] = onestep(w);
  ludcv.steptimes(i) = mean(diff([0; ts(indices)]));
  ludcv.steplengths(i) = 2*sin(get(w,'alpha'));
  ludcv.Ps(i) = get(w,'P');
end
actualbumps = ludcv.heights*std(ludcv.bumps,1);
% First verify that step times are about constant, as needed for speed
clf; plot(ludcv.heights*std(ludcv.bumps,1), ludcv.steptimes)
% Then plot 
clf; plot(ludcv.heights*std(ludcv.bumps,1), 0.5*ludcv.Ps.^2)
X = [(actualbumps)'.^2 ones(length(ludcv.heights),1)];
pfit = regress(0.5*ludcv.Ps'.^2, X);
hold on; plot(actualbumps,pfit(1)*actualbumps.^2+pfit(2),'r'); 
xlabel('bump size'); ylabel('Push-off work'); 
legend('Simulation', '\beta^2 fit');

% and uniformly distributed bumps
clear consvu
rng('default'); numsteps = 100; % same bumps as frand above
consvu.bumps = 2*rand(1, numsteps); consvu.bumps = consvu.bumps - mean(consvu.bumps); % uniform -1 to 1
consvu.heights = linspace(0, 0.05, 16); %consvu.steptimes = 0*consvu.heights;
w = wu;
consvu.Ps(1) = get(w, 'P'); P = consvu.Ps(1);
consvu.speeds(1) = gaitspeed(w);
opts = optimoptions(@fsolve, 'Display', 'off');
[xc,tc,xs,ts,energies,indices] = onestep(w);
consvu.energies{1} = energies;
consvu.steplengths(1) = 2*sin(get(w,'alpha'));

for i = 2:length(consvu.heights)
  w = set(w, 'bumps', consvu.bumps*consvu.heights(i));
  consvu.Ps(i) = fsolve(@(pee) getspeedp(w, pee, 0.4), P, opts);
  P = consvu.Ps(i);
  [xc,tc,xs,ts,energies,indices] = onestep(set(w, 'P', P));
  consvu.energies{i} = energies;
  td = ts(indices);
  steptimes = diff([0; td]);
  consvu.steplengths(i) = 2*sin(get(w,'alpha'));
  consvu.steptimes{i} = steptimes;
end
clf; hold all
plot(std(consvu.bumps,1)*consvu.heights, 0.5*consvu.Ps.^2); % this is work per step
xlabel('bump size'); ylabel('Push-off work per step'); title('Constant speed walking')
plot(ludcv.heights*std(ludcv.bumps,1), 0.5*ludcv.Ps.^2)
X = [(actualbumps)'.^2 ones(length(ludcv.heights),1)];
pfit = regress(0.5*ludcv.Ps'.^2, X);
plot(actualbumps,pfit(1)*actualbumps.^2+pfit(2),'r'); 
legend('Uniform', 'Up-down sim', '\beta^2 fit');

% and normally distributed bumps
clear consvn
rng('default'); numsteps = 100; % same bumps as frand above
consvn.bumps = 1*randn(1, numsteps); consvn.bumps = consvn.bumps - mean(consvn.bumps); % normal dist, std 1
consvn.bumps = consvn.bumps / std(consvn.bumps,1); 
consvn.heights = linspace(0, 0.025, 16); %consvn.steptimes = 0*consvn.heights;
w = wu;
consvn.Ps(1) = get(w, 'P'); P = consvn.Ps(1);
consvn.speeds(1) = gaitspeed(w);
opts = optimoptions(@fsolve, 'Display', 'off');
[xc,tc,xs,ts,energies,indices] = onestep(w);
consvn.energies{1} = energies;
consvn.steplengths(1) = 2*sin(get(w,'alpha'));

for i = 2:length(consvn.heights)
  w = set(w, 'bumps', consvn.bumps*consvn.heights(i));
  consvn.Ps(i) = fsolve(@(pee) getspeedp(w, pee, 0.4), P, opts);
  P = consvn.Ps(i);
  [xc,tc,xs,ts,energies,indices] = onestep(set(w, 'P', P));
  consvn.energies{i} = energies;
  td = ts(indices);
  steptimes = diff([0; td]);
  consvn.steplengths(i) = 2*sin(get(w,'alpha'));
  consvn.steptimes{i} = steptimes;
end
clf; hold all
plot(std(consvn.bumps,1)*consvn.heights, 0.5*consvn.Ps.^2); % this is work per step
plot(std(consvu.bumps,1)*consvu.heights, 0.5*consvu.Ps.^2); % this is work per step
xlabel('bump size'); ylabel('Push-off work per step'); title('Constant speed walking')
plot(ludcv.heights*std(ludcv.bumps,1), 0.5*ludcv.Ps.^2)
X = [(actualbumps)'.^2 ones(length(ludcv.heights),1)];
ludcv.pfit = regress(0.5*ludcv.Ps'.^2, X);
plot(actualbumps,ludcv.pfit(1)*actualbumps.^2+ludcv.pfit(2),'k--'); 
legend('Normal', 'Uniform', 'Up-down sim', '\beta^2 fit');

% relative costs by fitting the others
X = [(consvu.heights*std(consvu.bumps,1))'.^2 ones(length(consvu.heights),1)];
consvu.pfit = regress(0.5*consvu.Ps'.^2, X);
X = [(consvn.heights*std(consvn.bumps,1))'.^2 ones(length(consvn.heights),1)];
consvn.pfit = regress(0.5*consvn.Ps'.^2, X);
textinfo = sprintf('Relative costs normal : uniform : up-down\n%4.2f : %4.2f : 1\n',...
  consvn.pfit(1)/ludcv.pfit(1), consvu.pfit(1)/ludcv.pfit(1));
disp(textinfo)
text(0.025, 0.0109, textinfo) 
%% Cost of transport vs bumpiness
clf; hold all
plot(std(consvn.bumps,1)*consvn.heights, 0.5*consvn.Ps.^2./consvn.steplengths); % this is work per dist
plot(std(consvu.bumps,1)*consvu.heights, 0.5*consvu.Ps.^2./consvu.steplengths); % this is work per dist
xlabel('bump size'); ylabel('Push-off work per distance'); title('Constant speed walking')
plot(ludcv.heights*std(ludcv.bumps,1), 0.5*ludcv.Ps.^2./ludcv.steplengths)
X = [(actualbumps)'.^2 ones(length(ludcv.heights),1)];
ludcv.pvfit = regress(0.5*ludcv.Ps'.^2./ludcv.steplengths', X);
plot(actualbumps,ludcv.pvfit(1)*actualbumps.^2+ludcv.pvfit(2),'k--'); 
legend('Normal', 'Uniform', 'Up-down sim', '\beta^2 fit');

% relative costs by fitting the others
X = [(consvu.heights*std(consvu.bumps,1))'.^2 ones(length(consvu.heights),1)];
consvu.pvfit = regress(0.5*consvu.Ps'.^2./consvu.steplengths', X);
X = [(consvn.heights*std(consvn.bumps,1))'.^2 ones(length(consvn.heights),1)];
consvn.pvfit = regress(0.5*consvn.Ps'.^2./consvn.steplengths', X);
textinfo = sprintf('Relative costs normal : uniform : up-down\n%4.2f : %4.2f : 1\n',...
  consvn.pvfit(1)/ludcv.pvfit(1), consvu.pvfit(1)/ludcv.pvfit(1));
disp(textinfo)
text(0.025, 0.0109, textinfo) 


%% Distribution of work per step
% As bumpiness increases, the distibution of collision work changes.
% For minor bumpiness, the collisions are roughly normally distributed.
% But for larer bumps, the distribution becomes skewed. It becomes a 
% negatively sloped line, meaning that there are more
% frequent small collisions, and relatively few large (costly) collisions.
clf; subplot(121); hold all
for i = 2:2:length(consvu.heights)
  %hist(-[consvu.energies{i}.heelstrikework]); % use this to plot regular histogram
  [nelements,xcenters] = hist(-[consvu.energies{i}.heelstrikework]); 
  plot(xcenters, nelements);
  xlabel('collision work per step'); ylabel('distribution'); title('uniform bumpiness');
  %plot(mean(-[consvu.energies{i}.heelstrikework])*[1 1],get(gca,'ylim'),'--')
end
plot(normalenergies.pushoffwork*[1 1], get(gca,'ylim'), '--'); % smooth ground

subplot(122); hold all
for i = 2:2:length(consvn.heights)
  %hist(-[consvn.energies{i}.heelstrikework]); % use this to plot regular histogram
  [nelements,xcenters] = hist(-[consvn.energies{i}.heelstrikework]); 
  plot(xcenters, nelements);
  xlabel('collision work per step'); ylabel('distribution'); title('normal bumpiness');
  %plot(mean(-[consvn.energies{i}.heelstrikework])*[1 1],get(gca,'ylim'),'--')
end
plot(normalenergies.pushoffwork*[1 1], get(gca,'ylim'), '--'); % smooth ground

%% Work vs time
% Observation: Long step times seem to go with relatively lower collisions.
% Highest collisions seem to go with the nominal step times.

clf; subplot(121); hold all
for i = 2:length(consvu.heights)
  plot([consvu.steptimes{i}],-[consvu.energies{i}.heelstrikework],'.',...
    'MarkerSize', 10)
  xlabel('step times'); ylabel('collision work'); title('uniform bumpiness');
end
subplot(122); hold all
for i = 2:length(consvn.heights)
  plot([consvn.steptimes{i}],-[consvn.energies{i}.heelstrikework],'.',...)
    'MarkerSize', 10)
  xlabel('step times'); ylabel('collision work'); title('normal bumpiness');
end

%% Hodographs for two cases with similar standard deviations

% up-down vs uniform
w = set(wu, 'bumps', consvu.bumps*consvu.heights(end));
P = consvu.Ps(end);
clf; subplot(121); hodograph(set(w, 'P', P));

wud = gradsearch(set(wu, 'bumps', ...
  lupdown.bumps*std(consvu.bumps*consvu.heights(end),1)),[],'info',0);
[vxs,vys] = hodograph(wud);
hold on; plot(vxs([1:end 1]), vys([1:end 1]), 'r-', 'linewidth',2); hold off;
legend('uniform', 'up-down');
title('COM hodographs on uneven terrain (uniform)');

% up-down vs normal
w = set(wu, 'bumps', consvn.bumps*consvn.heights(end));
P = consvn.Ps(end);
subplot(122); hodograph(set(w, 'P', P));

wud = gradsearch(set(wu, 'bumps', ...
  lupdown.bumps*std(consvn.bumps*consvn.heights(end),1)),[],'info',0);
[vxs,vys] = hodograph(wud);
hold on; plot(vxs([1:end 1]), vys([1:end 1]), 'r-', 'linewidth',2); hold off;
legend('normal', 'up-down');
title('COM hodographs on uneven terrain (gaussian)');
  
%% Play around with optimization, the stuff below doesn't really work.
% ten steps with a bump in the middle
bumps = zeros(1,10); bumps(5) = 0.05;
wc = set(wul, 'bumps', bumps);
[xc,tcstar] = onestep(wul);
% clear sailing for ten steps
[xc,tc10] = onestep(set(wul, 'bumps', zeros(1,10)));
xstar = get(wul,'xstar');
targetthetadot = xstar(2); targettc = tc10; % after ten steps we want this time and this speed
targetP = get(wul,'P');

wul10 = set(wul,'bumps', zeros(1,10));

% a single bump in the road
bumps = zeros(1,10); bumps(5) = 0.05; ctrls = ones(1,10)*get(wul,'P'); 
onestep(set(wul,'bumps',bumps,'controls',ctrls))

% a single control impulse in the middle
bumps = zeros(1,10); ctrls = ones(1,10)*get(wul,'P'); ctrls(5) = ctrls(5) + 0.05;
onestep(set(wul,'bumps',bumps,'controls',ctrls))
onestep(set(wul,'bumps',zeros(1,10)),[],[],ctrls)
x0 = ones(1,10)*get(wul,'P'); % initial guess is no change
fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], []); % A, B, Aeq, beq, lb, ub

options = optimoptions('fmincon', 'algorithm', 'active-set');
speedandtimetarget(x0,wul10,targetthetadot, targettc)
optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],@(x)speedandtimetarget(x,wul10,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub


% now do same thing with a slight bump
nsteps =2;
targetthetadot = xstar(2); targettc = tcstar*nsteps; % after ten steps we want this time and this speed

%onebump = zeros(1,nsteps); onebump(floor(nsteps/2)) = 0.05;
onebump = sparse(1,floor(nsteps/2),0.05,1,nsteps);
x0 = ones(1,nsteps)*get(wul,'P'); % initial guess is no change
wul10b = set(wul,'bumps', onebump);
optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],@(x)speedandtimetarget(x,wul10b,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
clf
plot(1:nsteps,optctrls); hold all
plot(get(gca,'xlim'),[1 1]*get(wul,'P'))

%% What do you do when the step occurs at various locations?
options = optimoptions('fmincon', 'algorithm', 'active-set');
[xc,tcstar] = onestep(wul);
xstar = get(wul,'xstar');
targetthetadot = xstar(2);  % after N steps we want this speed again
targetP = get(wul,'P');
nsteps = 11; wherebump = [1 2 3 5 7 9 11]; targettc = tcstar*nsteps;
clf; hold all;
for i = 1:length(wherebump)
  bumps = sparse(1,wherebump(i),0.05,1,nsteps); % one bump somewhere
  x0 = ones(1,nsteps)*get(wul,'P'); % initial guess is no change
  w = set(wul,'bumps', bumps);
  optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],...
    @(x)speedandtimetarget(x,w,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
  [xc,tc,xs,ts,energies,indices] = onestep(w,[],[],optctrls);
  xrange = (1:nsteps) - wherebump(i); % 
  pxrange = xrange - 0.3; % plot push-offs offset to left
  plot(pxrange,0.5*optctrls.^2,'.');
  plot(xrange, -[energies.heelstrikework],'o');
  plot(xrange, 0.01*diff([0;ts(indices)])); 
end
plot(get(gca,'xlim'),[1 1]*0.5*get(wul,'P')^2,':')
xlabel('steps'); ylabel('work'); title('bump in middle')


%% Optimal push-off sequencing for a single bump in the road
% With varying amount of steps before or after
% Note that you can't compensate in a single step, because the timing is
% messed up; with one push-off you can compensate speed but the timing 
% must be off.
options = optimoptions('fmincon', 'algorithm', 'active-set');
[xc,tcstar] = onestep(wul);
xstar = get(wul,'xstar');
targetthetadot = xstar(2);  % after N steps we want this speed again
targetP = get(wul,'P');
clf; subplot(211); hold all;
nsteps = [ 2 3 5 10 20 30]; clear stepup
stepup.nsteps = nsteps;
for i = 1:length(nsteps)
  targettc = tcstar*nsteps(i);
  bumps = sparse(1,ceil((nsteps(i)+1)/2),0.05,1,nsteps(i)); % one bump in middle
  x0 = ones(1,nsteps(i))*get(wul,'P'); % initial guess is no change
  w = set(wul,'bumps', bumps);
  optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],...
    @(x)speedandtimetarget(x,w,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
  [xc,tc,xs,ts,energies,indices] = onestep(w,[],[],optctrls);
  stepup.w(i) = w; stepup.optctrls{i} = optctrls; stepup.bumps{i} = bumps;
  stepup.t{i} = ts;
  xrange = (1:nsteps(i)) - ceil((nsteps(i)+1)/2); % 
  pxrange = xrange - 0.3; % plot push-offs offset to left
  plot(pxrange,0.5*optctrls.^2,'.');
  plot(xrange, -[energies.heelstrikework],'o');
  plot(xrange, 0.01*diff([0;ts(indices)])); 
end
plot(get(gca,'xlim'),[1 1]*0.5*get(wul,'P')^2,':')
xlabel('steps'); ylabel('work'); title('step up')
%bar((1:nsteps(i))-ceil((nsteps(i)+1)/2),stepup.bumps{length(nsteps)})
% timing
% transform time to approximately match the steps
%[xc,tc,xs,ts,energies,indices] = onestep(w,[],[],optctrls);
%plot(ts,xs); hold off
%plot(diff([0;ts(indices)])); hold all
%plot(get(gca,'xlim'),tcstar*[1 1],':')

% single step down within many other steps
subplot(212); hold all; clear stepdown
stepdown.nsteps = nsteps;
for i = 1:length(nsteps)
  targettc = tcstar*nsteps(i);
  bumps = sparse(1,ceil((nsteps(i)+1)/2),-0.05,1,nsteps(i));
  x0 = ones(1,nsteps(i))*get(wul,'P'); % initial guess is no change
  w = set(wul,'bumps', bumps);
  optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],...
    @(x)speedandtimetarget(x,w,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
  [xc,tc,xs,ts,energies,indices] = onestep(w,[],[],optctrls);
  stepdown.w(i) = w; stepdown.optctrls{i} = optctrls; stepdown.bumps{i} = bumps;
  stepdown.t{i} = ts;
  xrange = (1:nsteps(i)) - ceil((nsteps(i)+1)/2); % 
  pxrange = xrange - 0.3; % plot push-offs offset to left
  plot(pxrange,0.5*optctrls.^2,'.');
  plot(xrange, -[energies.heelstrikework],'o');
  plot(xrange, 0.01*diff([0;ts(indices)])); 
end
plot(get(gca,'xlim'),[1 1]*0.5*get(wul,'P')^2,':')
xlabel('steps'); ylabel('work'); title('step down')

%%
% consider a control with pushoff and hip
onebump = sparse(1,floor(nsteps/2),0.05,1,nsteps);
x0 = [ones(1,nsteps)*get(wul,'P'); ones(1,nsteps)*0]; % initial guess is no change
wul10b = set(wul,'bumps', onebump);
optctrls = fmincon(@(x)0.5*sum(sum(x.^2)), x0, [], [], [], [], [], [],@(x)speedandtimetarget(x,wul10b,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
clf
plot(1:nsteps,optctrls); hold all
plot(get(gca,'xlim'),[1 1]*get(wul,'P'))
speedandtimetarget(x0,wul10b,targetthetadot,targettc)
onestep(wul10b,[],[],optctrls)



% no bumps, just hip control
nobumps = zeros(1,10);
onestep(set(wul,'bumps',nobumps), [],[],x0)
x0 = [ones(1,nsteps)*get(wul,'P'); ones(1,nsteps)*0]; % initial guess is no change
x0(3) = x0(3)- 0.05;
onestep(set(wul,'bumps',nobumps), [],[],x0)

%% Optimal control for random bumps compared to constant push-off
% We see that there's a modest economy improvement by using the
% optimal control, in this case about 1/3 improvement over the 
% penalty of walking on uneven vs flat ground.
clear optrand
rng('default'); numsteps = 30; % same bumps as frand above
bumps = 2*rand(1, numsteps); bumps = bumps - mean(bumps); % uniform -1 to 1
height  = 0.05; 
w = wul;
opts = optimoptions(@fsolve, 'Display', 'off');
[xc,tcstar,xs,ts,energies,indices] = onestep(w); xstar = get(w,'xstar');
Pstar = get(w,'P'); 
w = set(w, 'bumps', bumps*height);
P = fsolve(@(pee) getspeedp(w, pee, 0.4), get(w,'P'), opts);
[xc,tc,xs,ts,energies,indices] = onestep(set(w, 'P', P));
td = ts(indices);
optrand.r = struct('steptimes', diff([0; td]'), 'totalwork', 0.5*P^2*numsteps,...
 'xs', xs, 'ts', ts, 'energies', energies, 'indices', indices);
optrand.flat.totalwork = 0.5*Pstar^2*numsteps;

% and do an optimal control
targettc = tcstar*numsteps; targetthetadot = xstar(2);
x0 = ones(1,numsteps)*P; % initial guess is no change
wopt = set(wul,'bumps', bumps*height);
optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],...
    @(x)speedandtimetarget(x,wopt,targetthetadot, targettc),options); % A, B, Aeq, beq, lb, ub
[xc,tc,xs,ts,energies,indices] = onestep(wopt,[],[],optctrls);
td = ts(indices);
optrand.o = struct('steptimes', diff([0; td]'), 'totalwork', sum(0.5*optctrls.^2),...
 'xs', xs, 'ts', ts, 'energies', energies, 'indices', indices);
clf; plot(optrand.r.ts,optrand.r.xs,optrand.o.ts,optrand.o.xs) 
title(sprintf('total work equal push-off = %f, optimal = %f, flat = %f', ...
  optrand.r.totalwork, optrand.o.totalwork, optrand.flat.totalwork))
xlabel('time'); ylabel('states');  
fprintf('relative cost equal:flat = %f, optimal:flat = %f\n', ...
  optrand.r.totalwork/optrand.flat.totalwork, optrand.o.totalwork/optrand.flat.totalwork);

%% Control of change of speed
% See how many steps you want to take to go from 0.4 to 0.5 speed.
% The solution is to speed up smoothly over about three steps.
% If you have more than three steps to do it, then you should slow down
% smoothly before speeding up again. But always the control trajectory
% finishes with the same increase.
wul4 = findgaitspeed1(wul,'speed',0.4,'info',0);
wul5 = findgaitspeed1(wul,'speed',0.5,'info',0);
[xc,tc] = onestep(wul5); 
thetadotstar = xc(2); tcstar = tc;
opts = optimoptions(@fsolve, 'Display', 'off');
%options = optimoptions('fmincon', 'algorithm', 'active-set');
options = optimoptions('fmincon', 'algorithm', 'interior-point');%,'Display','iter');
nstepschange = [1 2 4 8 10 12 14]; clear speedd
speedd.nstepschange = nstepschange;
clf; hold all;
for i = 1:length(nstepschange)
  flats = zeros(1, nstepschange(i)); % no bumps here
  x0 = ones(1, nstepschange(i))*get(wul4,'P'); % initial guess is for 0.4
  w = set(wul4,'bumps',flats);
  optctrls = fmincon(@(x)0.5*sum(x.^2), x0, [], [], [], [], [], [],...
    @(x)speedtarget(x,w,thetadotstar),options); % A, B, Aeq, beq, lb, ub
  speedd.w(i) = set(w, 'controls', optctrls);
  [xc,tc,xs,ts,energies,indices] = onestep(speedd.w(i));
  speedd.t{i} = ts; speedd.x{i} = xs; speedd.bumps{i} = flats;
  speedd.steptimes{i} = diff([0; ts(indices)]');
  speedd.totalwork(i) = 0.5*sum(optctrls.^2);
  speedd.controls{i} = optctrls;
  plot(-length(flats)+1:0, optctrls,'color', mycolors(i,:));
  plot(-length(flats)+1:0, 0.5*speedd.steptimes{i},'--','color', mycolors(i,:));
  if nstepschange(i) == 1 % print with dots
    plot(-length(flats)+1:0, optctrls,'.','color', mycolors(i,:));
    plot(-length(flats)+1:0, 0.5*speedd.steptimes{i},'.','color', mycolors(i,:));
  end
end
plot(get(gca,'xlim'),0.5*[tcstar tcstar],':')
plot(0.1, get(wul5,'P'),'o')
xlabel('steps'); legend('push-off','step time');

%%

myctrls = [0 0 0.08 0.08 speedd.controls{3}];
bumps = [0 0 0 0 0 0 0 0];
onestep(set(wul,'bumps',bumps,'controls',myctrls))
sum(0.5*myctrls.^2)
clf; hodograph(set(wul,'bumps',bumps,'controls',myctrls))

speedd.totalwork(4)
speedd.controls{4}
hodograph(set(wul,'bumps',bumps,'controls',speedd.controls{4}))
  

%% SNOPT student version
i = 2;
flats = zeros(1, nstepschange(i)); % no bumps here
x0 = ones(1, nstepschange(i))*0.09;%*get(wul4,'P'); % initial guess is for 0.4
w = set(wul4,'bumps',flats);
global w speed2
speed2 = thetadotstar;

xlow = repmat(-Inf,nstepschange(i),1);
xupp = repmat( Inf,nstepschange(i),1);
Flow = [-Inf; 0];
Fupp = [Inf;  0];
%userfun = @(x)[0.5*sum(x.^2); snoptspeedtarget(x,w,thetadotstar)];

[xopt,Fopt,inform,xmul,Fmul] = snopt(x0', xlow, xupp, Flow, Fupp, 'snoptuserfun')


xlow = [-Inf; -Inf];
xupp = [Inf; Inf];
Flow = [-Inf];
Fupp = [Inf];
[xopt,Fopt,inform] = snopt([0;0], xlow, xupp, Flow, Fupp, 'testsnopt')
% this isn't working

%% NLOPT
i = 4;
flats = zeros(1, nstepschange(i)); % no bumps here
x0 = ones(1, nstepschange(i))*get(wul4,'P'); % initial guess is for 0.4
w = set(wul4,'bumps',flats);
opt.algorithm = NLOPT_LN_COBYLA;
opt.min_objective = @(x) 0.5*sum(x.^2);
opt.fc = {@(x) nloptspeedtarget(x,w,thetadotstar)};
opt.fc_tol = 1e-8;
opt.xtol_rel = 1e-4;
opt.verbose = 1;
[xopt, fmin, retcode] = nlopt_optimize(opt, x0)


%% walkrw2unevenlp
[xc,tcstar] = onestep(wul);
[xc,tcbump] = onestep(wul,[],0.06); % a bump
% make tmargin = tcstar-tcbump, so that the largest bump
wlp = set(walkrw2unevenlp,'tmargin',(tcstar-tcbump));
[xc,tc,xs,ts,energies,indices]=onestep(wlp,[],0.05);

rng('default')
numsteps = 100;
clear prand
prand.bumps = 2*rand(1, numsteps); prand.bumps = prand.bumps - mean(prand.bumps); % uniform -1 to 1
prand.heights = linspace(0, 0.05, 21); prand.steptimes = 0*prand.heights;
for i = 1:length(prand.heights)
  [xd,td,xs,ts,sis,es] = manybumps(wlp, [], prand.bumps*prand.heights(i));
  steptimes = diff([0; td]);
  prand.steptimes(i) = mean(steptimes);
end
% with a plot of average step times vs bump height
plot(prand.heights*std(prand.bumps,1), prand.steptimes,...
  frand.heights*std(frand.bumps,1), frand.steptimes,...
  lupdown.heights*std(lupdown.bumps,1), lupdown.steptimes)
xlabel('heights'); ylabel('times'); legend('random','up and down')
% the split push-off doesn't work that well; it's expensive and it craps
% out and fairly low bump height, mainly because hip work is so inefficient
% and doesn't help to speed up again.

%% Aggressive control. Control for dead-beat gait.
% Let's see each upcoming bump and anticipate it by pushing off to 
% achieve exact regular pendulum path. This is one step lookahead.
% How it works is this. Based on the bump, you know the post-collision
% stance angle, and from the fixed point you know the correct speed for
% that angle. Then back-compute what the collision would be and therefore
% what post-push-off speed is needed.


wlp0 = set(walkrw2unevenlp,'tmargin',0);
[xc,tc,xs,ts,energies,indices]=onestep(wlp0,[],0);
[xc,tc,xs,ts,energies,indices]=onestep(wlp0,[],0.05);

rng('default')
numsteps = 100;
clear prand0
prand0.bumps = 2*rand(1, numsteps); prand0.bumps = prand0.bumps - mean(prand0.bumps); % uniform -1 to 1
prand0.heights = linspace(0, 0.05, 21); prand0.steptimes = 0*prand0.heights;
for i = 1:length(prand0.heights)
  [xd,td,xs,ts,sis,es] = manybumps(wlp0, [], prand0.bumps*prand0.heights(i));
  steptimes = diff([0; td]);
  prand0.steptimes(i) = mean(steptimes);
end
% with a plot of average step times vs bump height
plot(prand0.heights*std(prand0.bumps,1), prand0.steptimes,...
  prand.heights*std(prand.bumps,1), prand.steptimes,...
  frand.heights*std(frand.bumps,1), frand.steptimes,...
  lupdown.heights*std(lupdown.bumps,1), lupdown.steptimes)
xlabel('heights'); ylabel('times'); legend('random','up and down')



