function [ splineObject ] = splineObject( elev_list, dist_list )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

spliney = fit(dist_list(:), elev_list(:), 'smoothingspline');

splineObject.height_fun = spliney;
splineObject.slope_fun = @(x) differentiate(spliney,x);


end
