function [ fit_fun ] = smoothen_interpolate( sample_pts, values )
%UNTITLED Summary of this function goes here
%   create a function that 
%   fits from NxD - matrix 'datapts' to NxV matrix 'values'
%   caution: current usage of loess surface limits D to 2
% 
% TODO: Provide interpolating function as a parameter for flexibility.

numpts = size(sample_pts,1);
assert(numpts == size(values,1), 'Not same amount of pts in data and values')

numdims = size(values,2);

for i = 1:numdims
    fitstruct.(char('fit' + string(i))) = ...
        fit(sample_pts, values(:,i), 'lowess');
end

    function output_vals = fitting_function(points)
        output_vals = zeros(size(points,1),numdims);
        for j = 1:numdims
            output_vals(:,j) = fitstruct.(char('fit' + string(j)))(points);
        end
    end

    fit_fun = @fitting_function;
end



