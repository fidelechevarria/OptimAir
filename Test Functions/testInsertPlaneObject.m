
clearvars
close all

fig = figure;
ax = axes;

% position = randi(50,25,3);
% attitude = randi(90,25,3);

position = [10 0 10];
attitude = [90 45 45];

insertPlaneObject(position,attitude)

grid
axis equal
axis vis3d
view(45,45)
set(ax, 'Ydir', 'reverse')