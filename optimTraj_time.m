
function [myf,myc,myceq] = optimTraj_time(params,WP)

%% Obtain new way-point sequence
[numOfWaypoints,~] = size(WP.north);
j = 0;
k = 0;
new_north = [];
new_east = [];
new_up = [];
for i = 1:(2*numOfWaypoints)-1
    if mod(i,2) == 1
        j = j + 1;
        new_north = [new_north WP.north(j)];
        new_east = [new_east WP.east(j)];
        new_up = [new_up WP.up(j)];
    else
        k = k + 1;
        new_north = [new_north params(k)];
        new_east = [new_east params(k+numOfWaypoints-1)];
        new_up = [new_up params(k+2*(numOfWaypoints-1))];
    end
end

%% Spline generation along way-point sequence
N = 500; % Number of uniformly distributed points along the curve parameter
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

%% Function outputs
myf = total_time; % Value to minimize
myc = [ ]; % if >0 params are not a valid solution
myceq = [ ]; % if =0 params are not a valid solution


