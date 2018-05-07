function [ gradients ] = create_gradients( input_coords, index_variable )
%CREATE_GRADIENT_FN create gradients from a time series
%   create gradients using non-uniform second order FD (see
% http://cfd.mace.manchester.ac.uk/twiki/pub/Main/TimCraftNotes_All_Access/cfd1-findiffs.pdf)

[~, i] = sort(index_variable);


gradients = zeros(size(input_coords));

num_pts = max(size(index_variable));

indep_var_plus0 = index_variable(i(1:(num_pts-2)));
indep_var_plus1 = index_variable(i(2:(num_pts-1)));
indep_var_plus2 = index_variable(i(3:num_pts));
dep_var_plus0 = input_coords(i(1:(num_pts-2)),:);
dep_var_plus1 = input_coords(i(2:(num_pts-1)),:);
dep_var_plus2 = input_coords(i(3:num_pts),:);

% isn't actually worth it to parallelize...
for j = 2:(num_pts-1)
    delta_x_0 = (indep_var_plus1(j-1) - indep_var_plus0(j-1));
%     assert(delta_x_0 > 0)
    delta_x_1 = (indep_var_plus2(j-1) - indep_var_plus1(j-1));
%     assert(delta_x_1 > 0)
    denominator = delta_x_0 * delta_x_1 * (delta_x_0 + delta_x_1);
    term_0 = -delta_x_1*delta_x_1 * dep_var_plus0(j-1,:);
    term_1 = (delta_x_1 *delta_x_1 - delta_x_0*delta_x_0) * dep_var_plus1(j-1,:);
    term_2 = delta_x_0 *delta_x_0 * dep_var_plus2(j-1,:);
    
    gradients(j,:) = (term_0 + term_1 + term_2) / denominator;
end

% Do endpoints 1st order
gradients(end,:) = (dep_var_plus1(end,:) - dep_var_plus0(end,:)) ...
    / (indep_var_plus1(end) - indep_var_plus0(end));

gradients(1,:) = (dep_var_plus2(1,:) - dep_var_plus1(1,:)) ...
    / (indep_var_plus2(1) - indep_var_plus1(1));

% Sort back into original order
gradients(i,:) = gradients;
end

