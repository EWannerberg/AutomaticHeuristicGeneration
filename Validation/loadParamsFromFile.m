function [ param_struct ] = loadParamsFromFile( filename )
%LOADPARAMSFROMFILE Summary of this function goes here
%   Detailed explanation goes here

text = fileread(filename);

param_struct = jsondecode(text);

end

