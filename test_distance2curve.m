
clearvars
close all

x = randi(100,1,4);
y = randi(100,1,4);
z = randi(100,1,4);

N = 500;

ppcurve = cscvn([x;y;z]);
space = linspace(ppcurve.breaks(1),ppcurve.breaks(end),N);
smooth = fnval(ppcurve,space);
smooth_x = smooth(1,:)';
smooth_y = smooth(2,:)';
smooth_z = smooth(3,:)';

u = randi(100,1,4);
v = randi(100,1,4);
w = randi(100,1,4);

figure
hold on
fnplt(ppcurve);
for i = 1:3
    tic
    [distance,closestPointInCurve] = distance2curve([smooth_x smooth_y smooth_z],[u(i) v(i) w(i)]);
    toc
    scatter3(u(i),v(i),w(i));
    scatter3(closestPointInCurve(1),...
             closestPointInCurve(2),...
             closestPointInCurve(3));
    plot3([u(i) closestPointInCurve(1)],...
          [v(i) closestPointInCurve(2)],...
          [w(i) closestPointInCurve(3)]);
end
hold off
grid
view(45,45)
axis equal
axis vis3d