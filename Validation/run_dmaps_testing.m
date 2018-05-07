%% Run a validation of a DM controller from MPC outfile to data

step_length = 3;
interpolateGradients = false;
% config_file = 'heightmaps/Bolzano_to_Innsbruck_res50.json';
config_file = 'data/3_cos.json';


%% Calculate diffusion maps
addpath('../DiffusionMaps&ClosedObs');

dmapsTesting;

%% Setup for interpolation
meanSlope = mean(data(:,slope_col));
slopeRange = max(data(:,slope_col)) - min(data(:,slope_col));
centerSlope = (data(:,slope_col)-meanSlope)/slopeRange;

meanForce = mean(data(:,force_col));
forceRange = max(data(:,force_col)) - min(data(:,force_col));
centerForce = (data(:,force_col)-meanForce)/forceRange;

mean_ins = [meanForce, meanSlope, meanSlope];
range_ins = [forceRange, slopeRange, slopeRange];

slopeLagData = constructTimeLagged(centerSlope, num_tl_pts, 0);

%% Construct controller
if interpolateGradients
    lagsToChoose = [30, 50];

    predictors = [centerForce(reducedIndices), slopeLagData(reducedIndices,lagsToChoose)];

    [in_fn, grad_dyn, out_fn] = create_closedobs_interps_grads(predictors, ...
        eigvects(:,2:3), data(reducedIndices,force_col));
    
    in_construct = @(indata) in_fn((indata - mean_ins)./range_ins);

    controller = @(state, model) getNextControlDMGrads(state,model,...
        {in_construct, grad_dyn, out_fn}, step_length);
else 
    lagsToChoose = [25, 65];
    
    predictors = [centerForce(reducedIndices), slopeLagData(reducedIndices,lagsToChoose)];
    
    [in_fn, dyn_fn, out_fn] = create_closedobs_interps(predictors, ...
        eigvects(:,2:4), data(reducedIndices,force_col));
    
    in_construct = @(indata) in_fn((indata - mean_ins)./range_ins);

    controller = @(state, model) getNextControlDMGrads(state,model,...
        {in_construct, grad_dyn, out_fn}, step_length);
end

%% run stuff
scenario_history = runSingleScenario(config_file,controller, 5000);
