function [ in_fn, dynamic_fn, out_fn ] = create_closedobs_interps( input_mat, diffmap_coord_mat, output_mat )
%CREATE_CLOSEDOBS_INTERPS Create the set of interpolating functions
%   creates interpolation functions input->dmaps->dmaps+1->output

in_fn = create_interp_fun(input_mat, diffmap_coord_mat);
dynamic_fn = ...
    create_interp_fun(diffmap_coord_mat(1:end-1,:),...
        diffmap_coord_mat(2:end,:));
out_fn = create_interp_fun(diffmap_coord_mat, output_mat);


end

