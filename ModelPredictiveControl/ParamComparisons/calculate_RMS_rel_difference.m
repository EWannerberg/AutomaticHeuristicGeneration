function [ output ] = calculate_RMS_rel_difference( datafile1, datafile2 )
%CALCULATE_RMS_DIFFERENCE Calculate RMS difference of two output datafiles
% ( for same stretch, supposedly)


data1 = importdata(datafile1);
data2 = importdata(datafile2);

num_datas = size(data1.data,2);
    
output = zeros(1, num_datas);

dist_col1 = strcmp(data1.colheaders, 'x_local');
dist_col2 = strcmp(data2.colheaders, 'x_local');

for col = 1:num_datas
    % calculate RMS difference, ignoring where data1.data is 0
    data2col = spline(data2.data(:,dist_col2),data2.data(:,col),data1.data(:,dist_col1));
    output(col) = (nanmean(((data1.data(:,col) - data2col ...
        )).^2).^0.5)/(max([data1.data(:,col);data2col]) - min([data1.data(:,col);data2col]));
end


end

