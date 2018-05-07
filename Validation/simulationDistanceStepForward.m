function [ model ] = simulationDistanceStepForward(model, control, end_distance)
%SIMULATIONSTEPFORWARD Summary of this function goes here
%   Detailed explanation goes here

stateVec = stateToVec(model.state);
options = odeset('Events', @reachEndEvent);
modControlSpan = [0 (end_distance - model.state.x)/model.params.v_init 2*(end_distance - model.state.x)/model.params.v_min];
[~, yvals,~,ye,~] = ode45(model.diffEq, modControlSpan, stateVec, options);

if length(ye) < 1
    disp('ERROR no distance crossing event occurred at t = ' ...
        + string(yvals(1,2)) + ' x = ' + string(yvals(1,3)) ...
        + ' to t = ' + string(yvals(end,2)) + ' x = ' + string(yvals(end,3)))
end

model.state = vecToState([control, yvals(end,2:end)]);


%% helper functions

    function [value, isterminal, direction] = reachEndEvent(~,y)
       value = y(3)-end_distance;
       isterminal = true;
       direction = [];        
    end

    function state_vec = stateToVec(state)
        state_vec = [state.force state.t state.x state.v ...
            state.acc state.Temp state.height state.cost state.slope];
    end

    function state = vecToState(state_vec)
        state.force = state_vec(1);
        state.t = state_vec(2);
        state.x = state_vec(3);
        state.v = state_vec(4);
        state.acc  = state_vec(5);
        state.Temp  = state_vec(6);
        state.height  = state_vec(7);
        state.cost  = state_vec(8);
        state.slope = state_vec(9);
    end

end

