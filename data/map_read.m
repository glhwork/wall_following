clear;
clc;

map = load('map.txt');
r = map(:,2);
theta = map(:,1);

param = load('line_param.txt');
k = param(:,1);
b = param(:,2);
min_x = param(:,3);
max_x = param(:,4);

for i = 1:size(map , 1)
    x(i) = r(i) * cos(theta(i));
    y(i) = r(i) * sin(theta(i));
end

figure(1);
hold on;
% axis([-10 12 -10 40]);
% axis equal;

% for i = 1:size(x, 2)
%     plot(x(i), y(i), 'r.');
%     pause(0.01);
% end
% 
plot(x,y, 'r.');
plot([0], [0], 'k.', 'MarkerSize', 20);

for i = 1:size(param, 1)
    min_y = k(i) * min_x(i) + b(i);
    max_y = k(i) * max_x(i) + b(i);

    plot([min_x(i) max_x(i)], [min_y, max_y], 'LineWidth', 1.5);
    pause(1);
end

