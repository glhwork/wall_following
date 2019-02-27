clear;
clc;

map = load('map.txt');
map_trans = load('map_trans.txt');
map_left = load('map_left.txt');
map_right = load ('map_right.txt');


figure (1);
hold on;
plot(map(:,1), map(:,2),'r.');
plot(map_trans(:,1), map_trans(:,2),'b.');

figure (2);
plot(map_left(:,1), map_left(:,2),'r.');
axis([-1 5 -2.5 2]);

figure(3);
plot(map_right(:,1), map_right(:,2),'r.');
axis([-1 5 -2.5 2]);