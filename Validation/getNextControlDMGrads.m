function [ upcoming_control, next_control_dist ] = getNextControlDMGrads( current_state, model, DM_funs, step_len )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if nargin < 4
    step_len = 50;
end

next_control_dist = current_state.x + step_len;

input_fun=DM_funs{1};
grad_fun=DM_funs{2};
out_fun=DM_funs{3};

height_295m_pred = model.slopeEq(current_state.x + 295);
height_500m_pred = model.slopeEq(current_state.x + 493);

grad_startpt =  input_fun(...
                [current_state.force, height_295m_pred, height_500m_pred]);
[~, y] = ode45(@(x,y) grad_fun(y')', [0,step_len], grad_startpt);
grad_endpt = y(end,:);

upcoming_control = ...
    out_fun(grad_endpt);

upcoming_control = min(upcoming_control, model.params.motor_max_power);
upcoming_control = max(upcoming_control, model.params.max_braking_power);

end

