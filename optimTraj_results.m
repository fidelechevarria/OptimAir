function optimTraj_results(params,WP,automaticFGlaunchIsActivated)

%% Obtain resultant way-point sequence
[numOfWaypoints,~] = size(WP.north);
j = 0;
k = 0;
m = 0;
WP_types = 0;
for i = 1:numOfWaypoints
    m = m+1;
    if i == numOfWaypoints
        % Do nothing
    elseif WP.segment_type(m) == 0
        WP_types = [WP_types 1 0];
    elseif WP.segment_type(m) == 1
        WP_types = [WP_types 1 1 1 0];
    end
end
new_size = numel(WP_types);
new_north = zeros(1,new_size);
new_east = zeros(1,new_size);
for i = 1:new_size
    if WP_types(i) == 0
        j = j + 1;
        new_north(i) = WP.north(j);
        new_east(i) = WP.east(j);
    elseif WP_types(i) == 1
        k = k + 1;
        new_north(i) = params(k);
        new_east(i) = params(k+numOfWaypoints-1);
    end
end
new_up = 20*ones(1,new_size);
new_up(4) = 20;

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
WP_info.index = WP_index;
WP_info.types = WP_types;

%% Time computation using dynamic model
propagatedState = optimTraj_dynamicModel(smooth_north,smooth_east,smooth_up);

%% Graphics generation
% 3D Graphical representation
f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f1,'northwest') % Move the GUI to the center of the screen.
hold on
scatter3(new_north,new_east,new_up,9,'r','filled')
scatter3(WP.north,WP.east,WP.up,9,'b','filled')
for j = 1:numel(WP_info.types)
    if WP_info.types(j) == 1
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
optimTraj_plotStates(propagatedState,WP_info);

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