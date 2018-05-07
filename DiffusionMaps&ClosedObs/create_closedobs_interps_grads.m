function [ in_fn, dynamic_fn, out_fn ] = create_closedobs_interps_grads( input_mat, diffmap_coord_mat, output_mat )
%CREATE_CLOSEDOBS_INTERPS Create the set of interpolating functions
%   creates interpolation functions input->dmaps->dmaps+1->output

in_fn = create_interp_fun(input_mat, diffmap_coord_mat);
dynamic_fn = ...
    smoothen_interpolate(diffmap_coord_mat,...
        create_gradients(diffmap_coord_mat, 1:max(size(diffmap_coord_mat))));
out_fn = create_interp_fun(diffmap_coord_mat, output_mat);


end

