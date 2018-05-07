function [ eigvects, eigvals ] = constructDMaps( kernel_matrix, num_components )
%CONSTRUCTDMAPS 
%   

% normalization factor 
invroot_rowsum = 1./sparse(sqrt(sum(kernel_matrix,2)));

% symmetrize for computationally more efficient solution
symmetric_kernel = diag(invroot_rowsum) * kernel_matrix * diag(invroot_rowsum);

% solve eigenproblem
[vects, vals] = computeEigs(symmetric_kernel, num_components);
eigvals = diag(vals);

% undo symmetric tranformation to get real eigvects
vects = diag(invroot_rowsum) * vects;

% normalize
eigvects = vects*diag(sparse(1./sqrt(sum(vects.^2,1))));

% sort into descending order
[eigvals, inds] = sort(eigvals, 'descend');
eigvects = eigvects(:, inds);

end

