function [params, lb, ub] = setInitialParamsNBoundsForRW_v2(numSteps, nominalP_RW, runNonLinModel, stepTimeNomRW, boundPushOffBegin, boundPushOffEnd)

%This function generates params; the initial guess of the push off impulse
%trajectory and upper and lower bound of the trajectory ub and lb,
%respectively.

%Currently, I use 4 times the nominal push off impulse whihc generally
%works but depending on the optimization you can change it. lb and ub are
%set to infinity and zero unless they are bounded with boundPushOffBegin
%and boundPushOffEnd

if ~runNonLinModel  %no nonlinear model in this script so this will always be 1 for now
    lb = [ ones(1,numSteps)*0]; 
    ub = [ ones(1,numSteps)*inf];    
    params(1,:) = ones(1,numSteps)*nominalP_RW*4;  %<----this is the nominal initial guess

    if ~isempty(boundPushOffBegin)
       params(1:boundPushOffBegin) = nominalP_RW; 
       lb(1:boundPushOffBegin) = nominalP_RW;
       ub(1:boundPushOffBegin) = nominalP_RW;
    end

    if boundPushOffEnd ~= 0
       params(boundPushOffEnd+1:end) = nominalP_RW; 
       lb(boundPushOffEnd+1:end) = nominalP_RW;
       ub(boundPushOffEnd+1:end) = nominalP_RW;
    end
else
    lb = [ ones(1,numSteps)*0; ones(1,numSteps)*stepTimeNomRW/50];  
    ub = [ ones(1,numSteps)*inf*nominalP_RW ; ones(1,numSteps)*stepTimeNomRW*50];    
    params = [ones(1,numSteps)*nominalP_RW*4; ones(1,numSteps)*stepTimeNomRW];

    if ~isempty(boundPushOffBegin)
       params(:,1:boundPushOffBegin) = [ones(1,boundPushOffBegin)*nominalP_RW; ones(1,boundPushOffBegin)*stepTimeNomRW]; 
       lb(:,1:boundPushOffBegin) = [ones(1,boundPushOffBegin)*nominalP_RW; ones(1,boundPushOffBegin)*stepTimeNomRW];
       ub(:,1:boundPushOffBegin) = [ones(1,boundPushOffBegin)*nominalP_RW; ones(1,boundPushOffBegin)*stepTimeNomRW];
    end

    if boundPushOffEnd ~= 0
    %                params(boundPushOffEnd+1:end,:) = [nominalP_RW; stepTimeNomRW];
    %                lb(boundPushOffEnd+1:end,:) = [nominalP_RW; stepTimeNomRW];
    %                ub(boundPushOffEnd+1:end,:) = [nominalP_RW; stepTimeNomRW];
    %                 keyboard;%just to check
       temp = length(params)-boundPushOffEnd;
       params(:,boundPushOffEnd+1:end) = [ ones(1,temp)*nominalP_RW; ones(1,temp)*stepTimeNomRW];
    %                lb(:, boundPushOffEnd+1:end) = [ones(1,temp)*nominalP_RW; ones(1,temp)*stepTimeNomRW];
    %                ub(:, boundPushOffEnd+1:end) = [ones(1,temp)*nominalP_RW; ones(1,temp)*stepTimeNomRW];
       lb(1, boundPushOffEnd+1:end) = [ones(1,temp)*nominalP_RW; ];
       ub(1, boundPushOffEnd+1:end) = [ones(1,temp)*nominalP_RW; ];
    end
end

  

    








