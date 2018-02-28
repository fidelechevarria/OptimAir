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

%% Compute arclength
arc = zeros(N-1,1);
for i = 1:N-1
    arc(i) = sqrt((smooth_north(i+1)-smooth_north(i)).^2+...
                  (smooth_east(i+1)-smooth_east(i)).^2+...
                  (smooth_up(i+1)-smooth_up(i)).^2);
end

%% Compute curvature
curvature = optimTraj_ppcurv(smoothTraj,N);

%% Time computation using dynamic model
V_old = zeros(N,1);
V_new = zeros(N,1);
phi = zeros(N,1);
L = zeros(N,1);
D = zeros(N,1);
alpha = zeros(N,1);
alpha_old = zeros(N,1);
lat_G = zeros(N,1);

% Initial conditions
V_old(1) = 92.95;
alpha_old(1) = 0.1;
T = 11000;

% Call dynamic_model
for i = 1:N-1
    [V_new(i),phi(i),L(i),D(i),alpha(i),lat_G(i)] = optimTraj_model2D(V_old(i),alpha_old(i),arc(i),T,abs(curvature(i)));
    V_old(i+1) = V_new(i);
    alpha_old(i+1) = alpha(i);
end

% Compute total time
time = zeros(N,1);
for i = 1:N-1
    time(i) = arc(i)/V_new(i);
end

total_time = sum(time);

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
title('Trajectory approximation')
[~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
axis equal
axis vis3d % Lock aspect ratio of axes
view(-45,45); % Azimuth and elevation of initial view (degrees)
xlabel('North')
ylabel('East')
zlabel('Up')

% 2D Graphical representation
f2 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f2,'northeast') % Move the GUI to the center of the screen.
hold on
plot(curvature)
hold off
grid
title('Trajectory curvature')
xlabel('Evaluation point')
ylabel('Curvature (1/localRadius) [m^-1]')

% Make figures visible.
f1.Visible = 'on';
f2.Visible = 'on';

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