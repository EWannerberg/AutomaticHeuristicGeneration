function [ time_lagged_matrix ] = constructTimeLagged( datapoints, num_pts, weight_kappa )
%CONSTRUCTTIMELAGGED Construct time-lagged variables 
%   Construct a matrix of time-lagged variables by appending each data
%   point by the subsequent points in the first dimension, weighted by
%   exp(-weight_kappa * i), where i is the difference in index between the
%   point and the subsequent added one

orig_size = size(datapoints);
new_num_rows = orig_size(1)-num_pts+1;



time_lagged_matrix = zeros(new_num_rows, orig_size(2) * num_pts);

for i = 1:num_pts
    time_lagged_matrix(:,((i-1)*orig_size(2)+1):(i*orig_size(2))) = ...
        datapoints(i:(new_num_rows + i - 1),:)*exp(-weight_kappa*(i-1));
end

end

