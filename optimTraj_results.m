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

%% Compute heading and pitch Euler angles
pitch = zeros(1,N-1);
heading = zeros(1,N-1);
for i = 1:N-1
    horiz_dist = sqrt((smooth_north(i+1)-smooth_north(i))^2 +...
                      (smooth_east(i+1)-smooth_east(i))^2);
    pitch(i) = atan2(smooth_up(i+1)-smooth_up(i),horiz_dist);
    heading(i) = atan2(smooth_north(i+1)-smooth_north(i),...
                              smooth_east(i+1)-smooth_east(i));
end

%% Eliminate steps in Euler angles
pitch_new = optimTraj_eliminateSteps(pitch);
heading_new = optimTraj_eliminateSteps(heading);

%% Time computation using dynamic model
V_old = zeros(N,1);
V_new = zeros(N,1);
roll = zeros(N,1);
L = zeros(N,1);
D = zeros(N,1);
alpha = zeros(N,1);
alpha_old = zeros(N,1);
lat_G = zeros(N,1);
Clalpha = zeros(N,1);
Cd0 = zeros(N,1);
timestep = zeros(N,1);
Qd = zeros(N,1);
Cl = zeros(N,1);
Cd = zeros(N,1);
pitch_dot = zeros(N-1,1);
heading_dot = zeros(N-1,1);

% Constant parameters
rho = 1.225;
S = 9.1;
K = 0.1779;
m = 750;
g = 9.81;

% Look-up tables for Clalpha and Cd0
Clalpha_xdata = [-3.14 -2.36 -1.57 -0.78 -0.6 -0.28 -0.25 -0.22 -0.2 0 0.2 0.22 0.25 0.28 0.6 0.78 1.57 2.36 3.14];
Clalpha_ydata = convforce([0.05 1.0 0.0 1.0 0.7 -1.12 -1.34 -1.4 -1.34 0.0 1.34 1.4 1.34 1.12 0.7 1.0 0.0 -1.0 0.05],'lbf','N');
Cd0_xdata = [-3.142 -1.57 -0.26 0 0.26 1.57 3.142];
Cd0_ydata = convforce([0.06 1.5 0.036 0.028 0.036 1.5 0.06],'lbf','N');

% Initial conditions
V_old(1) = 92.95;
alpha_old(1) = 0.1;
T = 10000;

% Dynamic_model
for i = 1:N-2
    timestep(i) = arc(i)/V_old(i);
    pitch_dot(i) = (pitch_new(i+1)-pitch_new(i))/timestep(i);
    heading_dot(i) = (heading_new(i+1)-heading_new(i))/timestep(i);
    Qd(i) = 0.5*rho*V_old(i)^2;
    Clalpha(i) = fixpt_interp1(Clalpha_xdata,Clalpha_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
    Cd0(i) = fixpt_interp1(Cd0_xdata,Cd0_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
    roll(i) = atan2(V_old(i)*heading_dot(i)*cos(pitch(i)),g*cos(pitch(i))+V_old(i)*pitch_dot(i));
    L(i) = m*g*cos(pitch(i))*cos(roll(i))+m*V_old(i)*(pitch_dot(i)*cos(roll(i))+heading_dot(i)*cos(pitch(i))*sin(roll(i)));
    Cl(i) = L(i)/(Qd(i)*S);
    D(i) = Qd(i)*S*(Cd0(i)+K*Cl(i)^2);
    Cd(i) = D(i)/(Qd(i)*S);
    V_new(i) = V_old(i)+timestep(i)*((T-D(i)-m*g*sin(pitch(i)))/m);
    alpha(i) = Cl(i)/Clalpha(i); % Cl0 considered zero
    lat_G(i) = (V_new(i)*heading_dot(i))/9.8056;
    V_old(i+1) = V_new(i);
    alpha_old(i+1) = alpha(i);
end

% Compute total time
time = zeros(N,1);
for i = 1:N-2
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
title(['Total time ' num2str(total_time) 's'])
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
plot(heading_dot)
hold off
grid
title('Heading rate')
xlabel('Evaluation point')
ylabel('Heading rate [rad/s^-2]')

f3 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f3,'southeast') % Move the GUI
hold on
plot(V_new)
hold off
grid
title('Velocity')
xlabel('Evaluation point')
ylabel('Velocity [m/s]')

f4 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f4,'southwest') % Move the GUI
hold on
plot(lat_G)
hold off
grid
title('Lateral G''s')
xlabel('Evaluation point')
ylabel('Lateral acceleration [m*s^-2]')

f5 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f5,'north') % Move the GUI
hold on
plot(rad2deg(roll))
hold off
grid
title('Roll angle')
xlabel('Evaluation point')
ylabel('Roll [º]')

f6 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f6,'east') % Move the GUI
hold on
plot(rad2deg(pitch_new))
hold off
grid
title('Pitch angle')
xlabel('Evaluation point')
ylabel('Pitch [º]')

f7 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
movegui(f7,'south') % Move the GUI
hold on
plot(rad2deg(heading_new))
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