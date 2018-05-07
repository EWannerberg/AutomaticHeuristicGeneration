function [FX, RES] = modified_loc_lin_regr(Y, X, EPS_MED_SCALE, dist_mat)
%LOCAL_LINEAR_REGRESSION Computes the local linear regression fit of a
%function
% [FX, RES] = local_linear_regression(Y, X, EPS_MED_SCALE) computes the
% local linear regression fit of a function Y = f(X)
% 
% Y is an n-long vector of the function output/response at n points
%
% X is an nxk matrix of the k-dimensional function input at n points
%
% EPS_MED_SCALE is the scale to use in the local linear regression kernel 
% the kernel will be a Gaussian with width median(distances)/eps_med_scale
%
% FX is an n-long vector of the local linear approximation to Y at the 
% n points of interest
% 
% RES is an n-long vector of the normalized leave-one-out cross validation 
% error at the n points of interest

% Code adapted from example code by Dsilva et. al.
% for  “Parsimonious Representation of Nonlinear Dynamical Systems 
% Through Manifold Learning: A Chemotaxis Case Study“, linked at
% http://ronen.net.technion.ac.il/publications/journal-publications/

% Code link:
% http://ronen.net.technion.ac.il/files/2016/07/DsilvaACHA.zip


% number of data points
n = size(X, 1);

% optional argument
if nargin < 4
    % local kernel matrix
    K = squareform(pdist(X));
    eps = median(K(:))/EPS_MED_SCALE;
    W = (exp(-K.^2 / eps^2));
    clear K
else 
    W = dist_mat;
end




warning('off', 'MATLAB:nearlySingularMatrix');

% compute local fit for each data point
L = zeros(n);
for i=1:n
    Xx = [ones(size(X,1),1) X-repmat(X(i,:), n, 1)];

    Xx2 = Xx'.*repmat(W(i,:), size(Xx, 2), 1);
    A = (Xx2*Xx)\Xx2;
    L(i,:) = A(1,:);
end

% functional approximation
FX = L*Y;

% leave-one-out cross-validation errors
RES = sqrt(mean((Y-FX).^2)) / std(Y);
