function [ output_mat ] = makeDistMatrix( input_mat )
%MAKEDISTMATRIX Basically squareform(pdist(input))

output_mat = squareform(pdist(input_mat));

end

