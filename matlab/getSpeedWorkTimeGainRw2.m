function output = getSpeedWorkTimeGainRw2(xs, ts, indices, energies, optctrls, bumps, nominalPushOffWorkRW, stepLenNomRW, stepTimeNomRW)


%this function returns the different speeds such as speeds at the mid stance
%or heel strike instants and corresponding time instants such as timeMS ferefering to time instants of mid stance. Also gets the work done.

%  Input: w is a walksw2 object, 
%       xs is state vector (stanleg angle and angular velocity), 
%       ts is the corresponding time vector
%       indices is the mid stance instances of the simulation at the end of each step (each step begins at middle stance the leg is up right and eds at mid stance)
%       energies is the energy structure that the pushoff and collison works
%       optctrls are the optimal push off impulses for each step, 
%       bumps is the the terrain heights expressed in angular perturbation of the stance leg, 
%       nominalPushOffWorkRW; nominal push off work (for 1 step) of the gait (walk model) being used for optimization 
%       stepLenNomRW; nominal step length of the gait (walk model) being used for optimization
%       stepTimeNomRW; nominal step time of the gait (walk model) being used for optimization

%  output is structure that contains several paremters belong to the optimal solution:
    %  po_work_temp; % push off work per step
    %  hs_work_temp; % heelstrike work per step
    %  optWork;   %total push off work
    %  RelWork;  %the difference between optWork and total nominal work of the same number of steps
    %  speedMS; %speed at mid stance instances
    %  speedHS; %speed at mid heel strike instances
    %  speedImu; %speed calculated as done for human wearing an inertila measurement unit (Imu)
    %  timeGainHS; %time gain at heel strike instances
    %  timeGainMS; %time gain at mid stance instances
    %  timeMS; %time gain at mid stance 
    %  timeHS; %time gain at heel strike
    %  onFoamMSindex; %the first mid stance instant on uneven terrain
    %  timeNormalized; %normalized simulation time with respect to the first mid stance instant on uneven terrain
    %  timeNormalizedHS; %normalized simulation time with respect to the first heel strike instant on uneven terrain
    %  timeMS_wrtHS; %ms indices of timeNormalizedHS
    %  stepTime; %time durations of each step
    
%     Osman Darici


NomWork = length(optctrls)*nominalPushOffWorkRW;
optWork = 0.5*sum(optctrls.^2);
RelWork = optWork-NomWork(end);

speedMS = -[xs(1,2); xs(indices, 2)]; %the speed at mid stance instances. Indices refers to mid stance instants how the simuatlion was run.

indicesHS = indices-10;
speedHS = xs(indicesHS,2); %the speed at heel strike instances
numSteps = length(bumps);

hs_work_temp = -nominalPushOffWorkRW; po_work_temp = nominalPushOffWorkRW; 

stepTime = [ts(indices(1)) diff(ts(indices))']; % step time is defined mid stance to mid stance
prevStepLen = stepLenNomRW; prevStepTime = stepTimeNomRW;
strideTime =    [];         
strideLength = [];   
for z = 1: numSteps                                 
    currentStepLen = sin(abs(xs(indicesHS(z),1))) +  sin(abs(xs(indicesHS(z)+1,1)));
    currentStepTime = stepTime(z);
    strideTime =    [strideTime currentStepTime+prevStepTime];         
    strideLength = [strideLength   currentStepLen +  prevStepLen];  
    
    prevStepTime = currentStepTime;
    prevStepLen = currentStepLen;
      
    hs_work_temp = [hs_work_temp energies(z).heelstrikework];
    po_work_temp = [po_work_temp energies(z).pushoffwork];
end 

speedImu = [stepLenNomRW/stepTimeNomRW strideLength./strideTime];  %stride length over stride time, this how the human speed is found in the human experiments ORG


% we need to find the first step index (middle stance instants) that is on the uneven terrain.
% Corresponding time point should be use to normalize time such that the
% first middle stance instant on the uneven terrain should correspond to
% time zero. Let's say we have a terrain 0 0 0 bump 0 0 0, which is 3 steps
% level before a step up and 3 steps level after. The 4th step should be
% the time zero so we can plot the parameters with respecto the before or
% after the uneven terrain
onFoamMSindex = find(bumps ~= 0); 
if ~isempty(onFoamMSindex)
    onFoamMSindex = onFoamMSindex(1);
end

if ~isempty(onFoamMSindex)
    timeNormalized = ts-ts(indices(onFoamMSindex));
    timeNormalizedHS = ts-ts(indicesHS(onFoamMSindex));
else
    timeNormalized = ts-ts(indices(7)); %<-----------hard coded
    timeNormalizedHS = ts-ts(indices(7));
end


% timeMS = [ts(1); ts(indices)]-ts(indices(onFoamMSindex));
timeMS = [timeNormalized(1); timeNormalized(indices)]; %finds the same above line


timeMS_wrtHS = [timeNormalizedHS(1); timeNormalizedHS(indices)]; %ms indices with HS instant zero on overall time, this is necessary to plot differnt speeds and steplengths figure, to sync them to HS instants


timeGainMS = -[0 ts(indices)' - cumsum(ones(1,numSteps)*stepTimeNomRW)]; %negative of time deviation of each step from cumulative normal time
xx = ts(indicesHS(2:end))' - ts(indicesHS(1));%for HS time gain we should begin from HS 2
timeGainHS = -[0 xx-cumsum(ones(1,numSteps-1)*stepTimeNomRW) ];

timeHS = [timeNormalized(indicesHS(1))-stepTimeNomRW; timeNormalized(indicesHS)];


%Nov 16 2022 we ant the real optimal data
po_work_temp = po_work_temp(2:end);
hs_work_temp = hs_work_temp(2:end);
speedMS = speedMS(2:end);
speedImu = speedImu(2:end);
timeGainMS = timeGainMS(2:end);
timeMS = timeMS(2:end);
timeMS_wrtHS = timeMS_wrtHS(2:end);
timeHS = timeHS(2:end);


output.po_work = po_work_temp; 
output.hs_work = hs_work_temp; 
output.optWork = optWork;   
output.RelWork = RelWork; 
output.speedMS = speedMS;
output.speedHS = speedHS; 
output.speedImu = speedImu;
output.timeGainHS = timeGainHS; 
output.timeGainMS = timeGainMS; 
output.timeMS = timeMS; 
output.timeHS = timeHS; 
output.onFoamMSindex = onFoamMSindex; 
output.timeNormalized = timeNormalized; %
output.timeNormalizedHS = timeNormalizedHS; 
output.timeMS_wrtHS = timeMS_wrtHS; 
output.stepTime = stepTime; 


end
