function [bumps, numSteps, optimationOnFlag, constantTimeFlag ]= setBumps2v3(numStepsBefore, numStepsAfter, terrainType, alpha, stdVal, offsetBump, uniformScale, bumpsHeightTraj)

% Inputs: numStepsBefore, numStepsAfter are the number of level seps before and after the uneven terrain
% terrainType is the uneven terrain type such oU only up (a one step up) or
% UnD up and down (one step up and then one step down)
% alpha is the leg angle (fixed for this model), 2*alpha is the interleg angle that determines the step length2*sin(alpha)
% stdVal, offsetBump, uniformScale are standart deviation, offset height
% and unformity to define different terrain trajectories (I have not really used these parameters)
% bumpsHeightTraj is the trajectory of the terrain 


%This function will generate the uneven terrain trajectory or bumps. 
%For example we want a 7 steps uneven terrain where the 4th step is a step
%up, then the output bumps should be [0 0 0 bumps_height 0 0 0]. Each
%number in this array is the angular difference that the stance leg will
%encounter. The default dimensionless bum height or bumHeightDimless is 0.075 which corresponds to 7.5 cm. Depending on
%the step length of rimless will the bumpHeight as an angle perturbation
%will be found. 

% numStepsBefore, numStepsAfter determine how many zero perturbation or
% level steps are there before the uneven terrain begins. For the array [0
% 0 0 bumps_height 0 0 0], both of these paremeters are 3. 3 level steps
% before and after the step up. 

% Note that if the array was [0 0 0 bumps_height -bumps_height 0 0], it
% would have a step up and a step down, with 3 level steps before and 2
% level steps after.

%The remaning parameters are to generate random terrains that I have not
%used before.


bumHeightDimless = 0.075;%this is like 7.5cm
bumps_height  = asin(bumHeightDimless/(2*sin(alpha)));

% for now
% one_inchAngle = 0.095/3; %BE CAREFUL ABOUT BUMP HEIGHT 
% bumps_height = 3*one_inchAngle;%
  
numSteps = numStepsBefore + numStepsAfter;

if ~isempty(strfind(terrainType,'_no_opt'))
    optimationOnFlag = 0;
else
    optimationOnFlag = 1;
end

if ~isempty(strfind(terrainType,'constantTime'))
    constantTimeFlag = 1;
else
    constantTimeFlag = 0;
end


if ~isempty(strfind(terrainType,'oD2FFsBump'))
     bumps = [zeros(1,numStepsBefore) -bumps_height, 0,  5/3*bumps_height, -5/3*bumps_height, zeros(1,numStepsAfter)]; %5/3 comes from experiment
     numSteps = length(bumps);
       return
elseif ~isempty(strfind(terrainType,'oU') )
       bumps = [zeros(1,numStepsBefore) bumps_height, zeros(1,numStepsAfter)];
       numSteps = length(bumps);
       return
elseif ~isempty(strfind(terrainType,'oD'))
       bumps = [zeros(1,numStepsBefore) -bumps_height, zeros(1,numStepsAfter)];
       numSteps = length(bumps);
       return    
elseif ~isempty(strfind(terrainType,'UnD'))
       bumps = [zeros(1,numStepsBefore),bumps_height, -bumps_height zeros(1,numStepsAfter)];
       numSteps = length(bumps);
       return
elseif ~isempty(strfind(terrainType,'DnU'))
       bumps = [zeros(1,numStepsBefore),-bumps_height, bumps_height zeros(1,numStepsAfter)];
       numSteps = length(bumps);
       return   
elseif ~isempty(strfind(terrainType,'longUnevenA'))
       bumpsHeightTraj = [3 2 -3 2 -1 3 1 -3 -2 3 -1 -2 -1 3 -2 -2]*bumps_height/3; %26 Oct 2020%<----used for Impulse reponse paper %<-----------
       bumps = [zeros(1,numStepsBefore), bumpsHeightTraj, zeros(1,numStepsAfter)];
       numSteps  =  length(bumps) ;
       return 
elseif ~isempty(strfind(terrainType,'longUnevenB'))
        bumpsHeightTraj = [3 2 -3 2 -1 3 1 -3 -2 3 -1 -2 -1 3 -2 -2]*bumps_height/3; %26 Oct 2020%<----used for Impulse reponse paper
        bumpsHeightTraj = -flip(bumpsHeightTraj);       
        bumps = [zeros(1,numStepsBefore), bumpsHeightTraj, zeros(1,numStepsAfter)];
        numSteps  =  length(bumps) ;
        return 
elseif ~isempty(strfind(terrainType,'pyramid'))
        bumpsHeightTraj = [3 3 3 0 0 0 -3 -3 -3]*bumps_height/3%pyramid
        bumps = [zeros(1,numStepsBefore), bumpsHeightTraj, zeros(1,numStepsAfter)];
        numSteps  =  length(bumps) ;     
        return
elseif ~isempty(strfind(terrainType,'level'))  
       bumps = [zeros(1,numStepsBefore), zeros(1, str2num(terrainType(end-1:end))), zeros(1,numStepsAfter)];
       numSteps  =  length( bumps) ;
       return 
elseif ~isempty(strfind(terrainType,'traj'))  
       bumps = [zeros(1,numStepsBefore), bumpsHeightTraj, zeros(1,numStepsAfter)];
       numSteps  =  length( bumps) ;
       return                            
elseif ~isempty(strfind(terrainType,'uniform'))    
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));                                                       %mean should be 0 
     unevenTerrainVal = -bumps_height*uniformScale + 2*bumps_height*uniformScale.*rand(1, numPertSteps);%this should create between -bumps_height +bumps_height, mean zero  
%      numSteps  = numSteps - 1 + numPertSteps ;
elseif ~isempty(strfind(terrainType,'normal'))
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));
     unevenTerrainVal = bumps_height+ stdVal.*randn(1, numPertSteps);
%      numSteps  = numSteps -1 + numPertSteps ;
elseif ~isempty(strfind(terrainType,'stairsUD'))
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));
     stairSteps = floor(numPertSteps/2);
     bumps =  [ zeros(1, numStepsBefore-1)  bumps_height*ones(1, stairSteps)   -bumps_height*ones(1, stairSteps) zeros(1, numStepsAfter) ] ; 
     numSteps = length(bumps);% numSteps - 1 + numPertSteps;
     return;
elseif ~isempty(strfind(terrainType,'stairsDU'))
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));
     stairSteps = floor(numPertSteps/2);
     bumps =  [ zeros(1, numStepsBefore-1)  -bumps_height*ones(1, stairSteps)   bumps_height*ones(1, stairSteps) zeros(1, numStepsAfter) ] ; 
     numSteps = length(bumps);%numSteps - 1 + numPertSteps;
     return;
elseif ~isempty(strfind(terrainType,'stairsU'))
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));
     bumps =  [zeros(1, numStepsBefore-1)  bumps_height*ones(1, numPertSteps) zeros(1, numStepsAfter)] ; 
     numSteps = length(bumps);%numSteps + numPertSteps;
     return;
elseif ~isempty(strfind(terrainType,'stairsD'))
     temp = strfind(terrainType,'_')+1;
     numPertSteps = str2num(terrainType(temp:end));
     bumps =  [zeros(1, numStepsBefore-1) -bumps_height*ones(1, numPertSteps) zeros(1, numStepsAfter)] ; 
     numSteps = length(bumps);%numSteps + numPertSteps;
     return;
else
    error('specify the the trial type');
end

%here we have either normal or uniform orclean them first
std1 = std(unevenTerrainVal);
badBumps1 = find(abs(unevenTerrainVal) > alpha); %0.3 is alpha if this is higher then alpha while stepping down then the model can not have positive angle the begining of the step
if ~isempty(badBumps1)
    unevenTerrainVal(badBumps1) = [];
    keyboard;
end

badBumps2 = find(abs(unevenTerrainVal) > 3*std1);
if ~isempty(badBumps2)
    unevenTerrainVal(badBumps2) = [];
    keyboard;
end
std2 = std(unevenTerrainVal);




if ~isempty(strfind(terrainType,'normal'))   
    unevenTerrainVal = unevenTerrainVal*stdVal/std2;
else
    unevenTerrainVal = unevenTerrainVal*sqrt(1/12 * (2*bumps_height*uniformScale)^2)/std2;%sqrt(1/12 * (2*bumps_height)^2) is std of uniform dist.
end
unevenTerrainVal = unevenTerrainVal - mean(unevenTerrainVal); %mean should be 0 for both 
%*************

bumps  = [zeros(1, numStepsBefore-1)  offsetBump+unevenTerrainVal zeros(1, numStepsAfter)];
numSteps = length(bumps);

