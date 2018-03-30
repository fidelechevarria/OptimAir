function results(params,WP,automaticFGlaunchIsActivated)

%% Obtain resultant way-point sequence
[numOfWaypoints,~] = size(WP.north);
j = 0;
k = 0;
new_size = numel(WP.WP_types);
new_north = zeros(1,new_size);
new_east = zeros(1,new_size);
for i = 1:new_size
    if WP.WP_types(i) == 0
        j = j + 1;
        new_north(i) = WP.north(j);
        new_east(i) = WP.east(j);
    elseif WP.WP_types(i) == 1
        k = k + 1;
        new_north(i) = params(k);
        new_east(i) = params(k+numOfWaypoints-1);
    end
end
% new_up = 20*ones(1,new_size);
new_up(1) = 30;
new_up(2) = 40;
new_up(3) = 50;
new_up(4) = 60;
new_up(5) = 70;
new_up(6) = 80;
new_up(7) = 90;
new_up(8) = 100;
new_up(9) = 110;
new_up(10) = 100;
new_up(11) = 90;
new_up(12) = 80;
new_up(13) = 70;
new_up(14) = 60;
new_up(15) = 50;
new_up(16) = 40;
new_up(17) = 30;

%% Spline generation along way-point sequence
N = 250; % Number of uniformly distributed points along the curve parameter
smoothTraj = cscvn([new_north;new_east;new_up]);
space = linspace(smoothTraj.breaks(1),smoothTraj.breaks(end),N);
smooth = fnval(smoothTraj,space);
smooth_north = smooth(1,:)';
smooth_east = smooth(2,:)';
smooth_up = smooth(3,:)';

%% Calculate cumulative number of control points for each segment
WP_index = [];
for i = 1:numel(smoothTraj.breaks)
    [~,index] = min(abs(space-smoothTraj.breaks(i)));
    WP_index = [WP_index index];
end
WP.WP_index = WP_index;

%% Time computation using dynamic model
propagatedState = dynamicModel(smooth_north,smooth_east,smooth_up);

%% Graphics generation
% 3D Graphical representation
f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f1,'northwest') % Move the GUI to the center of the screen.
hold on
scatter3(new_north,new_east,new_up,9,'r','filled')
scatter3(WP.north,WP.east,WP.up,9,'b','filled')
for j = 1:numel(WP.WP_types)
    if WP.WP_types(j) == 1
        text(new_north(j),new_east(j),new_up(j),num2str(j-1),'Color','r','VerticalAlignment','bottom')
    else
        text(new_north(j),new_east(j),new_up(j),num2str(j-1),'Color','b','VerticalAlignment','bottom')
    end
end
plot3(smooth_north,smooth_east,smooth_up)
hold off
grid
title(['Total time ' num2str(propagatedState.totalTime) 's'])
% [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
axis equal
axis vis3d % Lock aspect ratio of axes
% view(-45,45); % Azimuth and elevation of initial view (degrees)
xlabel('North')
ylabel('East')
zlabel('Up')

% 2D Graphical representation
plotStates(propagatedState,WP);

% Make figure visible.
f1.Visible = 'on';

%% FlightGear interface

% FG_initlat = 37.657773; %[decimal degrees]
% FG_initlon = -122.282163; %[decimal degrees]
% FG_initalt = 10; % [meters]
% FG_initpitch = 0; % [radians]
% 
% earthRadiusAtPoles = 6356725; % Do not modify this value
% earthRadiusAtEquator = 6378137; % Do not modify this value
% localEarthRadius = sqrt(((earthRadiusAtEquator^2*cos(FG_initlat))^2+...
%     (earthRadiusAtPoles^2*sin(FG_initlat))^2)/...
%     ((earthRadiusAtEquator*cos(FG_initlat))^2+...
%     (earthRadiusAtPoles*sin(FG_initlat))^2)); % [meters]
% 
% FG_lat = timeseries(FG_initlat+((y_coord-y_coord(1))/localEarthRadius)*180/pi,time);
% FG_lon = timeseries(FG_initlon+((x_coord-x_coord(1))/localEarthRadius)*180/pi,time);
% FG_alt = timeseries(FG_initalt*ones(numel(time),1),time);
% FG_pitch = timeseries(FG_initpitch*ones(numel(time),1),time);
% FG_roll = timeseries(phi,time);
% FG_yaw = timeseries(heading,time);
% 
% if automaticFGlaunchIsActivated == 1
%     system('optimFP_runFG &')
%     pause(30)
%     sim('optimFP_simulate_flight')
% end