function [stepLenNomRW, stepTimeNomRW, nominalP_RW, nominalLegSpeedRW, nominalPushOffWorkRW, nominalStepSpeedRW, nominalMiddleStanceTimeRW, nominalHeelStrikeWorkRW,...
    nominalHipTorqeWithoutPushOff, xsNomWithMS, tsNomWithMS, vintermsNom] = loadNominalsRW(varargin)%,w, speed_str


%inputs can be a walk object or a string that determines what nominal
%parameters to load. The function loads the variables into the workspace



if length(varargin) > 1
    w = varargin{1};
    speed_str = varargin{2};
    stepLenNomRW = 2*sin(get(w,'alpha'));
else
    speed_str = varargin{1};
    stepLenNomRW = 2*sin(0.3); %default step Len
end
    
load(strcat('stepTimeNomRW_',speed_str));     load(strcat('nominalStepSpeedRW_',speed_str));
load(strcat('nominalP_RW_',speed_str));       load(strcat('nominalMiddleStanceTimeRW_',speed_str))
load(strcat('nominalLegSpeedRW_',speed_str)); load(strcat('nominalHeelStrikeWorkRW_',speed_str));
load(strcat('nominalPushOffWorkRW_',speed_str));

% load(strcat('nominalHipTorqeWithoutPushOff_', speed_str)); %'1_40new'  speed_str
load(strcat('xsNomWithMS_', speed_str));
load(strcat('tsNomWithMS_', speed_str));
load(strcat('vintermsNom_', speed_str));
% load(strcat('xsNomAnimMSstate_', speed_str));

% vintermsNom = [];
nominalHipTorqeWithoutPushOff = [];
