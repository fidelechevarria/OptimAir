% States
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

% Controls
controls = [falcon.Control('alpha_dot',-10,10,0.1);
		    falcon.Control('T_dot',-9000,9000,1e-4);
            falcon.Control('p',-7.3,7.3,0.1)];
        
% Create the final time parameter
tf = falcon.Parameter('Final_Time',15,0,100,1e-2);

% Create the problem
problem = falcon.Problem('RBAR');

% Specify discretization
tau = linspace(0,1,101);

% Add a new phase
phase = problem.addNewPhase(@source_aircraft,states,tau,0,tf);
phase.addNewControlGrid(controls,tau);

% Set boundary condition
phase.setInitialBoundaries([5000;80;0.100000000000000;-0.500000000000000;0;0;0.866025403784439;0;0;20],...
                           [5000;80;0.100000000000000;-0.500000000000000;0;0;0.866025403784439;0;0;20]);
phase.setFinalBoundaries([0;40;-0.700000000000000;0.882947592858927;0;0;0.469471562785891;1.350718082030935e+02;2.672487733179268e+02;20],...
                         [9000;200;0.700000000000000;0.882947592858927;0;0;0.469471562785891;1.350718082030935e+02;2.672487733179268e+02;20]);

% Add cost function
problem.addNewParameterCost(tf);

% Path Constraint
pathconstraint = falcon.Constraint('quat_const', -inf, 0);
phase.addNewPathConstraint(@source_path, pathconstraint,tau);

% Solve problem
problem.Solve();

% Plot
figure
for numState=1:10
subplot(2,5,numState);
grid on;
hold on;
xlabel('time');
ylabel(phase.StateGrid.DataTypes(numState).Name);
plot(phase.RealTime,phase.StateGrid.Values(numState,:));
end