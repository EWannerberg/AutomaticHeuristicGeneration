function [ model ] = initialiseCarModel( param_struct )
%INITIALISECARMODEL Summary of this function goes here
%   Detailed explanation goes here

state.force = 0;
state.t = 0;
state.x = 0;
state.v = param_struct.v_init;
state.acc = 0;
state.Temp = param_struct.Temp_air;
state.height = 0;
state.cost = 0;
state.slope = 0;

model.labels = {'force', 'timepoint', 'x', 'v', 'acc', 'Temp', 'height', 'cost', 'slope'};
% model.timepoints = linspace(param_struct.episode_time_init, ...
%     param_struct.episode_time_end, ...
%     param_struct.episode_timesteps);
% model.controlDt = (param_struct.episode_time_end -  ...
%     param_struct.episode_time_init)/param_struct.episode_timesteps ...
%     * param_struct.dt_per_control_steps;
% model.controlSpan = [0 model.controlDt];


param_struct.cool_exponent = log(2)/param_struct.rad_cool_half_time;

% put a cost function here; outsource later
dcost_dtfn = @(~,state_vector) ...
    param_struct.motor_cost_coef * ...
    abs( subplus(state_vector(1)) * state_vector(4)) + ...
    param_struct.vel_cost_coef * param_struct.motor_max_power ...
    * 0.5 / (param_struct.v_max - param_struct.v_init) ...
    * (param_struct.v_init - state_vector(4))^2 + ...
    0; % param_struct.acc_cost_coef * state.acc;
model.dcost_dtfn = dcost_dtfn;

model.diffEq = @updateModel;

model.state = state;
model.params = param_struct;

[elev_list, dist_list] = loadHeightMap(param_struct.elevation_file);

% fouri = fourierObject(elev_list, dist_list);
spliney = splineObject(elev_list, dist_list);

% model.heightdata.heightObj = fouri;
model.heightdata.heightObj = spliney;
model.heightdata.elev_list = elev_list;
model.heightdata.dist_list = dist_list;

model.slopeEq = model.heightdata.heightObj.slope_fun;


    function [ f_diff_vector ] = updateModel(~, state_vector)
    % variables according to model.labels

    slope =     model.slopeEq(state_vector(3));
    air_res =   - param_struct.air_fric_coef * state_vector(4)^2;
    roll_fric = - param_struct.car_mass * param_struct.roll_fric_coef * param_struct.grav_const * (1 - abs(slope));
    gravity =   - slope * param_struct.car_mass * param_struct.grav_const;

    dTempDt = (subplus(state_vector(1)) * state_vector(4) * (1-param_struct.motor_heat_eff) ...
        - param_struct.air_cool_coef * state_vector(4) * (state_vector(6) - param_struct.Temp_air))/param_struct.motor_heat_capac ...
        - param_struct.cool_exponent * (state_vector(6) - param_struct.Temp_air);

    result_acc = (state_vector(1) + air_res + roll_fric + gravity)/param_struct.car_mass;
    
    f_diff_vector = [0; 1; state_vector(4); result_acc; 0; dTempDt; slope*state_vector(4); dcost_dtfn([],state_vector); 0];

    end
end
