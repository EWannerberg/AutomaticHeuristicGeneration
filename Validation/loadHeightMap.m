function [ elev_list, dist_list ] = loadHeightMap( filename )
%LOADHEIGHTMAP Summary of this function goes here
%   Detailed explanation goes here

addpath('readNPY');

dats = readNPY(filename);

elev_list = dats(1,:);
dist_list = dats(2,:);

end

