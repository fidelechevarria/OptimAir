%% Initial tasks
% Add falcon toolbox directory to path (only for UAV Nav MSI Laptop)
addpath('C:\Users\fidel\Desktop\falcon')

%% Create States
states = [falcon.State('T',0,9000,1e-4);
          falcon.State('V',40,200,0.005);
		  falcon.State('alpha',-0.7,0.7,1);
		  falcon.State('q0',-1,1,1);
          falcon.State('q1',-1,1,1);
	      falcon.State('q2',-1,1,1);
          falcon.State('q3',-1,1,1);
          falcon.State('x',-1e5,1e5,1e-3);
          falcon.State('y',-1e5,1e5,1e-3);
          falcon.State('h',5,2000,1e-3)];

%% Create Controls
controls = [falcon.Control('alpha_dot',-10,10,0.1);
		    falcon.Control('T_dot',-9000,9000,1e-4);
            falcon.Control('p',-7.3,7.3,0.1)];
        
%% Create phase times
Times = [falcon.Parameter('FinalTime1',15,0,100,1e-2);...
         falcon.Parameter('FinalTime2',27,0,100,1e-2)];

%% Create complete problem
problem = falcon.Problem('optimTraj');

%% Set major iteration limit
problem.setMajorIterLimit(5000);

%% Specify discretization
tau = linspace(0,1,101);

%% Create phases
% --------Phase 1---------
iP = 1;
FinalTime = Times(iP);
phase = problem.addNewPhase(@dynamicModel,states,tau,0,FinalTime);
phase.addNewControlGrid(controls,tau);

% Set boundary condition
phase.setInitialBoundaries([5000;80;0.100000000000000;-0.500000000000000;0;0;0.866025403784439;0;0;20],...
                           [5000;80;0.100000000000000;-0.500000000000000;0;0;0.866025403784439;0;0;20]);
phase.setFinalBoundaries([0;40;-0.700000000000000;0.882947592858927;0;0;0.469471562785891;1.350718082030935e+02;2.672487733179268e+02;20],...
                         [9000;200;0.700000000000000;0.882947592858927;0;0;0.469471562785891;1.350718082030935e+02;2.672487733179268e+02;20]);

% Path Constraint
pathconstraint = falcon.Constraint('quat_const', 0, 0);
phase.addNewPathConstraint(@pathConstraints, pathconstraint,tau);

%% --------Phase 2---------
iP = 2;
StartTime = Times(iP-1);
FinalTime = Times(iP);

phase = problem.addNewPhase(@dynamicModel,states,tau,StartTime,FinalTime);
phase.addNewControlGrid(controls,tau);

% Set boundary condition (only final)
phase.setFinalBoundaries([0;40;-0.700000000000000;-0.997565481207868;0;0;0.069736007216604;504.2889;772.4271;20],...
                         [9000;200;0.700000000000000;-0.997565481207868;0;0;0.069736007216604;504.2889;772.4271;20]);

% Path Constraint
pathconstraint = falcon.Constraint('quat_const', 0, 0);
phase.addNewPathConstraint(@pathConstraints, pathconstraint,tau);

%% Connect phases
problem.ConnectAllPhases();

%% Add cost function
problem.addNewParameterCost(Times(end));

%% Solve problem
problem.Solve();

%% Get Euler angles
q0 = problem.Phases(1).StateGrid.Values(4,:)';
q1 = problem.Phases(1).StateGrid.Values(5,:)';
q2 = problem.Phases(1).StateGrid.Values(6,:)';
q3 = problem.Phases(1).StateGrid.Values(7,:)';
eul = quat2eul([q0 q1 q2 q3],'ZYX');

%% Plot
figure
for numState=1:10
subplot(2,5,numState);
grid on;
hold on;
xlabel('time');
ylabel(problem.Phases(1).StateGrid.DataTypes(numState).Name);
plot(problem.Phases(1).RealTime,problem.Phases(1).StateGrid.Values(numState,:));
end

% 3D Graphical representation
f1 = figure;
movegui(f1,'northwest') % Move the GUI to the center of the screen.
ax = axes;
hold on
plot3(problem.Phases(1).StateGrid.Values(8,:),problem.Phases(1).StateGrid.Values(9,:),problem.Phases(1).StateGrid.Values(10,:));
hold off
grid
axis equal
axis vis3d % Lock aspect ratio of axes
view(-45,30); % Azimuth and elevation of initial view (degrees)
set(ax, 'Ydir', 'reverse')
xlabel('North')
ylabel('East')
zlabel('Up')

figure
for numState=1:3
subplot(3,1,numState);
grid on;
hold on;
xlabel('time');
plot(problem.Phases(1).RealTime,eul(:,numState).*180./pi);
end
