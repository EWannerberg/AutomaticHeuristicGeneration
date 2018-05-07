function [ upcoming_control, next_control_dist ] = getNextControlFromFile( current_state, model, controlFileName)
%GETNEXTCONTROL Summary of this function goes here
%   --- currently just setting a dummy value ---

persistent controlArray
persistent distanceArray
persistent lastReadFileName
persistent ctrlind

if ~strcmp(controlFileName, lastReadFileName)
    % read file
    if ~isempty(strfind(controlFileName, '.out')) && isempty(strfind(controlFileName, '.csvdata'))
        readFile = importdata(controlFileName);
        distanceCol = strcmp(readFile.colheaders, 'x_history') ...
            | strcmp(readFile.colheaders, 'x_local');
        motorForceCol = strcmp(readFile.colheaders, 'motor_force_history') ...
            | strcmp(readFile.colheaders, 'motor_force');
        brakingForceCol = strcmp(readFile.colheaders, 'braking_force_history') ...
            | strcmp(readFile.colheaders, 'braking_force');
        
        distanceArray = readFile.data(:,distanceCol);
        controlArray = readFile.data(:,motorForceCol) ...
            + readFile.data(:,brakingForceCol);
    else
        csvfile = csvread(controlFileName);
        controlCol = 2;%strcmp(model.labels, 'force');
        distanceCol = 4;%strcmp(model.labels, 'x');
        
        controlArray = csvfile(:, controlCol);
        distanceArray = csvfile(:, distanceCol);
    end
    
    lastReadFileName = controlFileName;
    ctrlind = 1;
end

epsil = 0.01;

if abs(current_state.x - distanceArray(ctrlind + 1)) > epsil
    %TODO: handle exceptional cases: distance not find, etc.
    ctrlind = find(distanceArray > current_state.x + epsil, 1) - 1;
    if ctrlind == 0
        ctrlind = 1;
    end
else 
    ctrlind = ctrlind + 1;
end

try
    upcoming_control = controlArray(ctrlind + 1);
    next_control_dist = distanceArray(ctrlind + 1);
catch E
    disp(E)
    upcoming_control = controlArray(end);
    next_control_dist = inf;
end

end
