close all; clear;
p = [0, 1, 2, 0, 2, -1, 2]';
v = [0, 0, 1, 0, 2, -1, 0]';
t = [0, 1, 2, 3, 4, 5, 7]';

PVT_queue = [p, v, t]; % PVT_queue is a N x 3 matrix, N presents the PVT key datapoints nums, and 3 presents the dimensions.
clear p v t

PVT_param_queue = PVTSpline(PVT_queue); % PVT_param_queue is a N x 4 matrix, N presents the datapoints nums, and 4 presents the dimensions of a,b,c,d.

data_point_num = 1000;
PVT_spline = zeros(data_point_num, 3);
t = linspace(-2,10,data_point_num)';
for index=1:data_point_num
    PVT_spline(index, :) = getPVTValue(PVT_param_queue, PVT_queue, t(index));
end

figure
hold on;
plot(PVT_spline(:,3), PVT_spline(:,1));
plot(PVT_queue(:,3), PVT_queue(:,1), 'r*');
title('Demo of PVT spline');
xlabel('t');
ylabel('p');
hold off;
