function data = readNPY(filename)
% Function to read NPY files into matlab. 
% *** Only reads a subset of all possible NPY files, specifically N-D arrays of certain data types. 
% See https://github.com/kwikteam/npy-matlab/blob/master/npy.ipynb for
% more. 
%

[shapeOfIn, dataType, fortranOrder, littleEndian, totalHeaderLength, ~] = readNPYheader(filename);

if littleEndian
    fid = fopen(filename, 'r', 'l');
else
    fid = fopen(filename, 'r', 'b');
end

try
    
    [~] = fread(fid, totalHeaderLength, 'uint8');

    % read the data
    data = fread(fid, prod(shapeOfIn), [dataType '=>' dataType]);
    
    if length(shapeOfIn)>1 && ~fortranOrder
        data = reshape(data, shapeOfIn(end:-1:1));
        data = permute(data, [length(shapeOfIn):-1:1]);
    elseif length(shapeOfIn)>1
        data = reshape(data, shapeOfIn);
    end
    
    fclose(fid);
    
catch me
    fclose(fid);
    rethrow(me);
end
