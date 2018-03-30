
clearvars
close all

fig = figure;
position = randi(30,5,3);

insertPlaneObject(fig,position)

grid
axis equal
axis vis3d
view(45,45)

