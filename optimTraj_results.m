function optimTraj_results(params,WP,automaticFGlaunchIsActivated)

%% Obtain resultant way-point sequence
[numOfWaypoints,~] = size(WP.north);
j = 0;
k = 0;
complete_size = (2*numOfWaypoints)-1;
complete_north = zeros(1,complete_size);
complete_east = zeros(1,complete_size);
result_size = length(params)/2;
result_north = zeros(1,result_size);
result_east = zeros(1,result_size);
for i = 1:complete_size
    if mod(i,2) == 1
        j = j + 1;
        complete_north(i) = WP.north(j);
        complete_east(i) = WP.east(j);
    else
        k = k + 1;
        complete_north(i) = params(k);
        complete_east(i) = params(k+numOfWaypoints-1);
        result_north(i) = params(k);
        result_east(i) = params(k+numOfWaypoints-1);
    end
end
complete_up = 20*ones(1,complete_size);
result_up = 20*ones(1,result_size);

%% Spline generation along way-point sequence
N = 250; % Number of uniformly distributed points along the curve parameter
smoothTraj = cscvn([complete_north;complete_east;complete_up]);
space = linspace(smoothTraj.breaks(1),smoothTraj.breaks(end),N);
smooth = fnval(smoothTraj,space);
smooth_north = smooth(1,:)';
smooth_east = smooth(2,:)';
smooth_up = smooth(3,:)';

%% Time computation using dynamic model
propagatedState = optimTraj_dynamicModel(smooth_north,smooth_east,smooth_up);

%% Graphics generation
% 3D Graphical representation
f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f1,'northwest') % Move the GUI to the center of the screen.
hold on
scatter3(complete_north,complete_east,complete_up,9,'r','filled')
scatter3(WP.north,WP.east,WP.up,9,'b','filled')
plot3(smooth_north,smooth_east,smooth_up)
hold off
grid
title(['Total time ' num2str(propagatedState.total_time) 's'])
% [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
axis equal
axis vis3d % Lock aspect ratio of axes
% view(-45,45); % Azimuth and elevation of initial view (degrees)
xlabel('North')
ylabel('East')
zlabel('Up')

% 2D Graphical representation
f2 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f2,'northeast') % Move the GUI
hold on
plot(propagatedState.heading_dot)
hold off
grid
title('Heading rate')
xlabel('Evaluation point')
ylabel('Heading rate [rad/s^-2]')

f3 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f3,'southeast') % Move the GUI
hold on
plot(propagatedState.V_new)
hold off
grid
title('Velocity')
xlabel('Evaluation point')
ylabel('Velocity [m/s]')

f4 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f4,'southwest') % Move the GUI
hold on
plot(propagatedState.lat_G)
hold off
grid
title('Lateral G''s')
xlabel('Evaluation point')
ylabel('Lateral acceleration [m*s^-2]')

f5 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f5,'north') % Move the GUI
hold on
plot(rad2deg(propagatedState.roll))
hold off
grid
title('Roll angle')
xlabel('Evaluation point')
ylabel('Roll [º]')

f6 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f6,'east') % Move the GUI
hold on
plot(rad2deg(propagatedState.pitch_new))
hold off
grid
title('Pitch angle')
xlabel('Evaluation point')
ylabel('Pitch [º]')

f7 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f7,'south') % Move the GUI
hold on
plot(rad2deg(propagatedState.heading_new))
hold off
grid
title('Yaw angle')
xlabel('Evaluation point')
ylabel('Yaw [º]')

% Make figures visible.
f1.Visible = 'on';
f2.Visible = 'on';
f3.Visible = 'on';
f4.Visible = 'on';
f5.Visible = 'on';
f6.Visible = 'on';
f7.Visible = 'on';

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