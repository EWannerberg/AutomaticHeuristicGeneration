% example of using local linear regression to find unique eigendirections
% for a TUM logo

% heavily inspired and adapted from example code by Dsilva et. al.
% for  “Parsimonious Representation of Nonlinear Dynamical Systems 
% Through Manifold Learning: A Chemotaxis Case Study“, linked at
% http://ronen.net.technion.ac.il/publications/journal-publications/

% Code link:
% http://ronen.net.technion.ac.il/files/2016/07/DsilvaACHA.zip

% Add parent directory where (hopefully) the diffusion maps code is
[thisfolder,~,~] = fileparts(mfilename('fullpath'));
addpath([thisfolder, '/..'])

%% define initial parameters

% number of data points
N = 2000; 

% construct archemedian spiral
a = 1; 
theta_vec = linspace(0, 4*pi, 100);
s = 0.5*a*(theta_vec.*sqrt(1+theta_vec.^2)+log(theta_vec+sqrt(1+theta_vec.^2)));

% height
h = 20;

% number of diffusion maps eigenvectors to compute
neigs = 10;

%% generate data

% intialize random number generator
rng(321);

% find angles which correspond to uniform sampling along spiral
theta = interp1(s, theta_vec, rand(N, 1)*max(s));

% data uniformly distributed on swiss roll
% z = h*rand(N,1); 
% x = a * cos(theta) .* theta;
% y = a * sin(theta) .* theta;

tum_pts = generate_TUM_points(3000);
tum_pts = tum_pts*diag([80, 34, 40.5]./(max(tum_pts) - min(tum_pts)));

x = tum_pts(:,1);
y = tum_pts(:,2);
z = tum_pts(:,3);

% store all data
data = [x y z]; 

%% diffusion maps
% pairwise distance matrix
W = makeDistMatrix(data);

% kernel scale
eps = 4*median(min(W + eye(size(W))*max(max(W))));

kernel_mat = constructKernelMat(W,eps);
kernel_mat = normalizeKernel(kernel_mat);

% compute embedding coordinates
[V, D] = constructDMaps(kernel_mat, neigs);

%% local linear regression

% regression kernel scale
eps_med_scale = 3;

% compute cross-validation error in fitting diffusion maps eigenvectors 
% as a function of previous eigenvectors
res = compute_residuals_DMAPS(V, eps_med_scale);


% create cool colormap
colmap = colormap('hsv');
colmap = colmap(1:ceil(max(size(colmap))*0.73),:);

%% make plots
% some plot extras from http://flyingv.ucsd.edu/azad/Matlab_LaTeX_Figures.pdf
% plot original data
figure;
plot3(x,y,z,'.')
view(-30, 70)
xlabel('x')
ylabel('y')
zlabel('z')
set(gca,...
'Units','normalized',...
'Position',[.05 .05 .9 .8]...
,'FontUnits','points',...
'FontWeight','normal',...
'FontSize',9)%,...
axis equal
grid on
title('Original data')

% plot diffusion maps eigenvalues, colored by cross-validation error
figure;
colored_bars(D, res)
set(gca, 'ylim', [0 1])
xlabel('k')
ylabel('\mu_k')
axis square
colorbar
title('Diffusion maps eigenvalues, colored by cross-validation error')

% plot data colored by first identified unique eigendirection
% firstfig=figure('Units','points',...
% 'Position',[1 1 216 162],...
% 'PaperPositionMode','auto');
figure
colormap(colmap)
scatter3(x,y,z,50,V(:,2),'.')
set(gca,...
'Units','normalized',...
'Position',[.05 .05 .9 .8]...
,'FontUnits','points',...
'FontWeight','normal',...
'FontSize',9)%,...
% 'FontName','Times'...
view(-30, 70)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on
% colorbar
% color_minmax = caxis;
% color_minmax_new = color_minmax;
% color_minmax_new(2) = color_minmax(1) + (color_minmax(2) - color_minmax(1)) * 1.37;
% caxis(color_minmax_new);
colaxis = caxis;
title('Colored by first eigendirection (k=2)')

% plot data colored by second identified unique eigendirection
figure;
colormap(colmap)
scatter3(x,y,z,50,V(:,5),'.')
set(gca,...
'Units','normalized',...
'Position',[.05 .05 .9 .8]...
,'FontUnits','points',...
'FontWeight','normal',...
'FontSize',9)%,...
view(-30, 70)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on
colorbar
caxis(colaxis);
title('Colored by second eigendirection (k=5)')

% plot data colored by identified repeated eigendirection
figure;
colormap(colmap)
scatter3(x,y,z,50,V(:,3),'.')
set(gca,...
'Units','normalized',...
'Position',[.05 .05 .9 .8]...
,'FontUnits','points',...
'FontWeight','normal',...
'FontSize',9)%,...
view(-30, 70)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on
colorbar
title('Data colored by first repeated eigendirection (k=3)')
