function [ normalized_kernel_mat ] = normalizeKernel( kernel_matrix, alpha_exp )
%NORMALIZEKERNEL perform normalization
% This makes the diffusion maps eigenvectors equivalent to the heat kernels
% on the manifold.

if nargin < 2
    alpha_exp = 1;
end

density_estimate = sum(kernel_matrix,2);

multvector = diag(sparse(density_estimate.^(-alpha_exp)));

normalized_kernel_mat = multvector * kernel_matrix * multvector;


end

