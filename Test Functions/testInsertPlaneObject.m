
clearvars
close all

fig = figure;

% position = randi(50,25,3);
% attitude = randi(90,25,3);

position = [0 0 0];
attitude = [0 0 90];

insertPlaneObject(position,attitude)

grid
axis equal
axis vis3d
view(45,45)

