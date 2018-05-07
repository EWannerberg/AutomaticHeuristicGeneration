function [ interp_fun ] = create_interp_fun( datapt_mat, values_mat )
%CREATE_INTERP_FUN 
%   create a function that 
%   interpolates from NxD - matrix 'datapts' to NxV matrix 'values'
%   caution: current usage of scatteredInterpolant limits D to 3
% 
% TODO: Provide interpolating function as a parameter for flexibility.

numpts = size(datapt_mat,1);
assert(numpts == size(values_mat,1), 'Not same amount of pts in data and values')

numdims = size(values_mat,2);

for i = 1:numdims
    interpstruct.(char('interp' + string(i))) = ...
        scatteredInterpolant(datapt_mat, values_mat(:,i), 'natural');
end

    function output_vals = interpolation_function(points)
        output_vals = zeros(size(points,1),numdims);
        for j = 1:numdims
            output_vals(:,j) = interpstruct.(char('interp' + string(j)))(points);
        end
    end

    interp_fun = @interpolation_function;
end

