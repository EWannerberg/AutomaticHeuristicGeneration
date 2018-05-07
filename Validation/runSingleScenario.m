function [ historyarray ] = runSingleScenario( param_filename, controller, status_every )
%RUNSINGLESCENARIO run using controller
%   Run a single scenario using specified controller

% optional argument
if nargin < 3
    status_every = inf;
    if nargin < 2
        % if no controller specified, use a 50m-PID
        controller = @(state, model) deal(getNextControlPID50m(state, model), state.x + 50);
    end
end

filename = param_filename;

params = loadParamsFromFile(filename);
model = initialiseCarModel(params);

currentState = model.state;

next_output = status_every;
historyarray = [];

simulationIsFinished = false;
while ~simulationIsFinished
    
    [upcoming_control, nextCtrlDist] = controller(model.state, model);
    model = simulationDistanceStepForward(model, upcoming_control, nextCtrlDist);
    
    historyarray = [historyarray; stateToVec(model.state)];

    simulationIsFinished = model.state.x > model.heightdata.dist_list(end-50);
    simulationIsFinished = simulationIsFinished || model.state.v < 0.01; % -> something's gone wrong
    if model.state.x > next_output
        disp(['Validation currently at x = ', num2str(model.state.x)]);
        next_output = next_output + status_every;
    end
end

% utility for saving history
function state_vec = stateToVec(state)
        state_vec = [state.force state.t state.x state.v ...
            state.acc state.Temp state.height state.cost state.slope];
end

end

