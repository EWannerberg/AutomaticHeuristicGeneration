%% Define parameters
dataFile = '../MPC-data/timelag300_18000_sorted_2x_id.out';

num_eigs = 30;
% takeEvery = 1;
reducedIndices = 1:10000;

%% Import data
dataStruct = importdata(dataFile);
dataStruct.textdata = char(dataStruct.textdata);
dataStruct.colheaders = split(dataStruct.textdata(3:end),','); % starts with '# ' due to numpy

dist_col = find(strcmp(dataStruct.colheaders, 'x_local'));
force_col = find(strcmp(dataStruct.colheaders, 'motor_force'));
id_col = find(strcmp(dataStruct.colheaders, 'dataf_id'));
slope_col = find(strcmp(dataStruct.colheaders, 'slope'));

% ignore_rows = createSlopeFilter(dataStruct.data, slope_col, dist_col, 40, 0.2, id_col);
data = dataStruct.data(:,:);
timeLaggedData = data(:,(max(size(dataStruct.colheaders))+1):end);

%% Execute Diffusion Maps algorithm
dist_mat = makeDistMatrix(timeLaggedData(reducedIndices,:));
kernel_width = 13; %median(min(dist_mat + eye(size(dist_mat))*max(max(dist_mat))))*4;
kernel_mat = constructKernelMat(dist_mat, kernel_width);
kernel_mat = normalizeKernel(kernel_mat);
[ eigvects, eigvals ] = constructDMaps( kernel_mat, num_eigs );


%% Postprocess and plot
[~,inds] = sort(data(reducedIndices,dist_col));
figure; plot(data(reducedIndices(inds),dist_col),eigvects(inds,[2:10]))

%compute gradients once for each file used
gradients = zeros(size(eigvects(:,2:end)));
for id = reshape(unique(data(:,id_col)),1,[])
    this_id = data(reducedIndices,id_col) == id;
    gradients(this_id,:) = create_gradients(eigvects(this_id,2:end),data(reducedIndices(this_id),dist_col));
end

if ~(exist('k', 'var') && exist('l', 'var') && exist('m', 'var'))
    k = 2; l = 3; m = 4;
end
figure; q = quiver(eigvects(:,k),eigvects(:,l), gradients(:,k-1), gradients(:,l-1), 3);
mags = sqrt(sum(gradients(:,[k,l]).^2,2));
color_arrows(q, mags)
xlabel(k)
ylabel(l)

figure; q = quiver3(eigvects(:,k),eigvects(:,l), eigvects(:,m), gradients(:,k-1), gradients(:,l-1), gradients(:,m-1), 3);
mags = sqrt(sum(gradients(:,[k,l,m]).^2,2));
color_arrows(q, mags)
xlabel(k)
ylabel(l)
zlabel(m)
