function [ output_args ] = color_arrows(plot_handle, color_values )
%COLOR_ARROWS Color arrows according to values and colormap
% from https://stackoverflow.com/questions/29632430/quiver3-arrow-color-corresponding-to-magnitude

% Get the current colormap
currentColormap = colormap(gca);

% Now determine the color to make each arrow using a colormap
[~, ~, ind] = histcounts(color_values, size(currentColormap, 1));

% Now map this to a colormap to get RGB
cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
cmap(:,:,4) = 255;
cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);

% We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
set(plot_handle.Head, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap(1:3,:,:), [], 4).'); 

% We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
set(plot_handle.Tail, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap(1:2,:,:), [], 4).');
end
