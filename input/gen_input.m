close all;
tablemap = readtable('../maps/map5.txt');
map = table2array(tablemap);

fout = fopen('in5.txt', 'w');

figure(); % 'units','normalized','outerposition',[0 0 1 1]);
%imagesc(map); axis square; colorbar; colormap(c_map); hold on;
z_scale = 10;
hold on;
[X, Y] = meshgrid(1:size(map, 1), 1:size(map, 2));
s = surf(X, Y, z_scale * map);
s.EdgeColor = 'none';
colorbar;

ThreeD = -1;

while(ThreeD ~= 1 && ThreeD ~= 0)
    ThreeD = input('3D? (input 1 or 0)\n> ');
end

d = datacursormode;

input('click start');
Start = getfield(getCursorInfo(d),'Position');

Start_th = input('start angle (0 to 15)\n> ');

Start = [Start, Start_th];
pause;
d = datacursormode;
input('click goal');
Goal = getfield(getCursorInfo(d),'Position');

Goal_th = input('goal angle (0 to 15)\n> ');
Goal = [Goal, Goal_th];

Map_num = input('map num?\n> ');

ThreeD
Start
Goal
Map_num

%write the header
fprintf(fout, 'ThreeD: %d\n', ThreeD);
fprintf(fout, 'Start: %d %d %d %d\n', Start);
fprintf(fout, 'Goal: %d %d %d %d\n', Goal);
fprintf(fout, 'Map Num: %d\n', Map_num);

