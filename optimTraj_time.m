
function [myf,myc,myceq] = optimTraj_time(params,WP)
tic
%% Obtain new way-point sequence
[numOfWaypoints,~] = size(WP.north);
j = 0;
k = 0;
new_size = (2*numOfWaypoints)-1;
new_north = zeros(1,new_size);
new_east = zeros(1,new_size);
for i = 1:new_size
    if mod(i,2) == 1
        j = j + 1;
        new_north(i) = WP.north(j);
        new_east(i) = WP.east(j);
    else
        k = k + 1;
        new_north(i) = params(k);
        new_east(i) = params(k+numOfWaypoints-1);
    end
end
new_up = 20*ones(1,new_size);

%% Spline generation along way-point sequence
N = 250; % Number of uniformly distributed points along the curve parameter
smoothTraj = cscvn([new_north;new_east;new_up]);
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

% Constraints
max_lat_G = max(lat_G);
max_lat_G_limit = 10; % Maximum lateral acceleration in G's

% Compute total time
time = zeros(N,1);
for i = 1:N-2
    time(i) = arc(i)/V_new(i);
end

total_time = sum(time);
    
%% Function outputs
myf = total_time; % Value to minimize
myc = [max_lat_G-max_lat_G_limit]; % if >0 params are not a valid solution
myceq = [ ]; % if =0 params are not a valid solution
toc
