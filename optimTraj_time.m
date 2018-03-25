
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

%% Time computation using dynamic model
propagatedState = optimTraj_dynamicModel(smooth_north,smooth_east,smooth_up);

% Constraints
max_lat_G = max(propagatedState.lateralGForce);
max_lat_G_limit = 10; % Maximum lateral acceleration in G's
    
%% Function outputs
myf = propagatedState.totalTime; % Value to minimize
myc = [max_lat_G-max_lat_G_limit]; % if >0 params are not a valid solution
myceq = [ ]; % if =0 params are not a valid solution
toc
