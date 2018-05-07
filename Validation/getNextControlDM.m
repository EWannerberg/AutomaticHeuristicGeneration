function [ upcoming_control, next_control_dist ] = getNextControlDM( current_state, model, DM_funs, step_len )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if nargin < 4
    step_len = 50;
end

next_control_dist = current_state.x + step_len;

input_fun=DM_funs{1};
dyn_fun=DM_funs{2};
out_fun=DM_funs{3};

height_250m_pred = model.slopeEq(current_state.x + 250);
height_650m_pred = model.slopeEq(current_state.x + 650);

upcoming_control = ...
    out_fun(...
        dyn_fun(...
            input_fun(...
                [current_state.v, height_250m_pred, height_650m_pred])));


end

