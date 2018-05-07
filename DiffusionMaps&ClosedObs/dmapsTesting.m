%% Define parameters
dataFile = '../Validation/data/3_cos2017-07-03-10.43.10.out';

num_tl_pts = 150;
num_eigs = 30;
ignore_first = 300;
max_allowed_slope = 0.3;
takeEvery = 5;
tlag_end_attenuation = 0.2;
kernel_width_minmedians = 6;
% kernel_width_maxmins = 1.5;
% kernel_width_medians = 2;

%% Import data
dataStruct = importdata(dataFile);
data = dataStruct.data(ignore_first:end, :);


force_col = find(strcmp(dataStruct.colheaders, 'motor_force'));
dist_col = find(strcmp(dataStruct.colheaders, 'x_local'));
slope_col = find(strcmp(dataStruct.colheaders, 'slope'));

num_rows = max(size(data));

%% create filter for slope
highslope_cols = abs(data(:,slope_col)) > max_allowed_slope;
ignore_rows = zeros(size(highslope_cols), 'logical');

ignore_width = 250;
for i=1:ignore_width
    ignore_rows(1:(num_rows-i + 1)) = ignore_rows(1:(num_rows-i + 1)) | highslope_cols (i:num_rows);
    ignore_rows(i:num_rows) = ignore_rows(i:num_rows) | highslope_cols (1:(num_rows-i + 1));
end

%% construct time-lagged manifold
tlag_exponential_factor = -log(tlag_end_attenuation)/(num_tl_pts-1);
timeLaggedData = constructTimeLagged(zscore(data(:,force_col)), num_tl_pts, tlag_exponential_factor);

indices = 1:size(timeLaggedData,1);
% indices = indices(~ignore_rows(floor(ignore_width/2):(num_rows+floor(ignore_width/2)-num_tl_pts)));
indices = indices(~ignore_rows(1:(num_rows+1-num_tl_pts)));
reducedIndices = indices(1:takeEvery:max(size(indices)));

%% Execute Diffusion Maps algorithm
dist_mat = makeDistMatrix(timeLaggedData(reducedIndices,:));
kernel_width = median(min(dist_mat + eye(size(dist_mat))*max(max(dist_mat))))*kernel_width_minmedians;
% kernel_width = max(min(dist_mat + eye(size(dist_mat))*max(max(dist_mat))))*kernel_width_maxmins;
% kernel_width = median(dist_mat(:)) * kernel_width_medians;
kernel_mat = constructKernelMat(dist_mat, kernel_width);
kernel_mat = normalizeKernel(kernel_mat);
[ eigvects, eigvals ] = constructDMaps( kernel_mat, num_eigs );

%% Postprocess and plot
figure; plot(dataStruct.data(reducedIndices,dist_col),eigvects(:,[2:10]))
grads = create_gradients(eigvects(:,2:30),dataStruct.data(reducedIndices,dist_col));

if ~(exist('k', 'var') && exist('l', 'var') && exist('m', 'var'))
    k = 2; l = 3; m = 4;
end

figure; q = quiver(eigvects(:,k),eigvects(:,l), grads(:,k-1), grads(:,l-1), 3);
mags = sqrt(sum(grads(:,[k,l]).^2,2));
color_arrows(q, mags)
xlabel(['{\bf\psi}_', num2str(k)])
ylabel(['{\bf\psi}_', num2str(l)])

figure; q = quiver3(eigvects(:,k),eigvects(:,l), eigvects(:,m), grads(:,k-1), grads(:,l-1), grads(:,m-1), 3);
mags = sqrt(sum(grads(:,[k,l,m]).^2,2));
color_arrows(q, mags)
xlabel(['{\bf\psi}_', num2str(k)])
ylabel(['{\bf\psi}_', num2str(l)])
zlabel(['{\bf\psi}_', num2str(m)])
