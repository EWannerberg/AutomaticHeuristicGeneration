function [ plot_handle ] = plot_epsilon_curve( dist_mat, eps_start,eps_stop, num_steps )
%PLOT_EPSILON_CURVE Summary of this function goes here
%   Detailed explanation goes here

if nargin == 3
    num_steps = 10;
end

eps_steps = exp(log(eps_start):(log(eps_stop/eps_start)/num_steps):log(eps_stop));


results = zeros(size(eps_steps));

for i = 1:max(size(eps_steps))
    kernel_mat = exp(-(dist_mat/eps_steps(i)).^2);
    results(i) = sum(kernel_mat(:));
end

plot_handle = figure;
loglog(eps_steps, results);

end

