
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

% Constraints
max_lat_G = max(lat_G);
max_lat_G_limit = 10; % Maximum lateral acceleration in G's

% Compute total time
time = zeros(N,1);
for i = 1:N-1
    time(i) = arc(i)/V_new(i);
end

total_time = sum(time);

%% Function outputs
myf = total_time; % Value to minimize
myc = [max_lat_G-max_lat_G_limit]; % if >0 params are not a valid solution
myceq = [ ]; % if =0 params are not a valid solution
toc
