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
abs_curvature = abs(curvature);

%% Time computation using dynamic model
V_old = zeros(N,1);
V_med = zeros(N,1);
V_new = zeros(N,1);
phi = zeros(N,1);
L = zeros(N,1);
D = zeros(N,1);
alpha = zeros(N,1);
alpha_old = zeros(N,1);
lat_G = zeros(N,1);
Clalpha = zeros(N,1);
Cd0 = zeros(N,1);

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
T = 5000;

% Call dynamic_model
for i = 1:N-1
    Clalpha(i) = fixpt_interp1(Clalpha_xdata,Clalpha_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
    Cd0(i) = fixpt_interp1(Cd0_xdata,Cd0_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
    V_med(i) = optimTraj_model2D(V_old(i),arc(i),T,abs_curvature(i),Cd0(i),rho,S,K,m,g);
    V_new(i) = max(2*(V_med(i)-V_old(i))+V_old(i),1);
    phi(i) = acos(1/sqrt(V_new(i)^2*curvature(i)/g+1));
    Cl(i) = m*g/(0.5*rho*S*V_new(i)^2*cos(phi(i)));
    Cd(i) = Cd0(i) + K*Cl(i)^2;
    L(i) = 0.5*rho*V_new(i)^2*S*Cl(i);
    D(i) = 0.5*rho*V_new(i)^2*S*Cd(i);
    alpha(i) = Cl(i)/Clalpha(i); % Cl0 considered zero
    lat_G(i) = (V_new(i)^2*curvature(i))/9.8056;
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
movegui(f2,'northeast') % Move the GUI
hold on
plot(curvature)
hold off
grid
title('Trajectory curvature')
xlabel('Evaluation point')
ylabel('Curvature (1/localRadius) [m^-1]')

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

% Make figures visible.
f1.Visible = 'on';
f2.Visible = 'on';
f3.Visible = 'on';
f4.Visible = 'on';

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