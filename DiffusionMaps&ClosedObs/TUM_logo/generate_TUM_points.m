function [ tum_points  ] = generate_TUM_points( num_points )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

rng();

disallowed = @(x,y) ((y < 265) & ((x < 60) | ((120 < x) & (x < 180)) | ((370 < x) & (x < 440)) ...
    | ((500 < x) & (x < 570)))) | ((y > 60) & ((240 < x) & (x < 310)));


datapts = rand(2*num_points, 3) * diag([630, 325, 60]);

datapts = datapts(~disallowed(datapts(:,1),datapts(:,2)),:);

tum_points = datapts(1:num_points, :);


end

