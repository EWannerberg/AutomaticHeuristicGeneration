function [ kernel_matrix ] = constructKernelMat(dist_matrix, kernel_width)
%CONSTRUCTKERNELMAT Build the kernel matrix using the gaussian kernel.

kernel_matrix = exp(-(dist_matrix.^2)./(kernel_width.^2));

end

