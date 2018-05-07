function [ control ] = getNextControlPID50m( currentState, model )
%GETNEXTCONTROLPID Summary of this function goes here
%   Detailed explanation goes here

persistent lastState;
persistent inte;

if isempty(lastState)
    lastState = currentState;
    inte = 0;
end


steadyStateForce = 950;

intHalfTime = 40;

% intWeight = 0.5^(1/intHalfTime);
intWeight = 1;

prop = model.params.v_init - currentState.v;
if lastState.t ~= currentState.t
    diff = (currentState.v - lastState.v)/(currentState.t - lastState.t);
else 
    diff = 0;
end
inte = inte*intWeight + (model.params.v_init - currentState.v)*(currentState.t - lastState.t);

coef_P = 2;
coef_I = 0.6;
coef_D = 0.1;

scaling = 200;

control = steadyStateForce + (coef_P * prop + coef_I * inte + coef_D * diff) * scaling;

lastState = currentState;
end

