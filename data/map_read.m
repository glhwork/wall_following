clear;
clc;

laser = load('laser_scan.txt');
% laser_1 = load('laser_scan_1.txt');
x = laser(:,1);
y = laser(:,2);

% x_1 = laser_1(:,1);
% y_1 = laser_1(:,2);

% figure(1);
% hold on;
% axis([-1 3 -3 4]);
% for i = 1:size(x,1)
%     plot(x(i), y(i), 'r.');
% %     pause(0.05);
% end

% figure(2)
% hold on;
% axis([-3 3 -3 3]);
% for i = 1:size(x_1,1)
%     plot(x_1(i), y_1(i), 'r.');
% %     pause(0.05);
% end



x_vec = [];
y_vec = [];
index = 1;
count = 0;

angle_limit = 0.175;
least_n = 10;
angle_vec = [];

x_vec_tmp = [x(1) x(2)];
y_vec_tmp = [y(1) y(2)];
for i = 3:size(x,1)-2
    v1 = [x(index+1)-x(index) y(index+1)-y(index)];
    v2 = [x(i)-x(index) y(i)-y(index)];
    val = abs(dot(v1, v2) / (norm(v1) * norm(v2)));
    angle = acos(val);
    angle_vec = [angle_vec angle];
    if angle <= angle_limit
        x_vec_tmp = [x_vec_tmp x(i)];
        y_vec_tmp = [y_vec_tmp y(i)];
    elseif length(x_vec_tmp) < least_n
        x_vec_tmp = [];
        y_vec_tmp = [];
        x_vec_tmp = [x(i)];
        y_vec_tmp = [y(i)];
        index = i;
    end
    if angle > angle_limit && length(x_vec_tmp) > least_n
        count = count + 1;
        figure(count);
        plot(x_vec_tmp, y_vec_tmp, 'r.');
        x_vec_tmp = [];
        y_vec_tmp = [];
        x_vec_tmp = [x(i)];
        y_vec_tmp = [y(i)];
        index = i;
    end
        
end
if length(x_vec_tmp) > least_n
    count = count + 1;
    figure(count);
    plot(x_vec_tmp, y_vec_tmp, 'r');
end

figure(count+1)
k = 1:length(angle_vec);
plot(k, angle_vec, 'r.');

angle_plot = load('angle.txt');
j = 1:length(angle_plot);
figure(count+2);
hold on;
axis([0 145 0 2]);
plot(j', angle_plot(:,2), 'r.');
plot([0 140], [0.175 0.175]);











