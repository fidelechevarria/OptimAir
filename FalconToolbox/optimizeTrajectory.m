
function problem = optimizeTrajectory(configuration)

%% Create States
states = [falcon.State('vx',-200,200,1e-2);
          falcon.State('vy',-200,200,1e-2);
		  falcon.State('vz',-200,200,1e-2);
          falcon.State('roll',-5,5,1);
          falcon.State('pitch',-5,5,1);
          falcon.State('yaw',-5,5,1);
		  falcon.State('p',-8,8,1);
          falcon.State('q',-8,8,1);
	      falcon.State('r',-8,8,1);
          falcon.State('x',5,2000,1e-3);
          falcon.State('y',5,2000,1e-3);
          falcon.State('h',5,2000,1e-3)];

%% Create Controls
controls = [falcon.Control('da',-0.35,0.35,1);
		    falcon.Control('de',-0.35,0.35,1);
            falcon.Control('dr',-0.35,0.35,1);
            falcon.Control('dt',0,1,1)];
        
%% Create phase time
FinalTime = falcon.Parameter('FinalTime',5,0,50,0.1);

%% Create complete problem
problem = falcon.Problem('optimTraj');

%% Set major iteration limit
problem.setMajorIterLimit(5000);

%% Specify discretization
tau = linspace(0,1,251);

%% Create phase

% Phase
phase1 = problem.addNewPhase(@dynamicModel,states,tau,0,FinalTime);

% Control grid
phase1.addNewControlGrid(controls,tau);

% Set boundary condition
phase1.setInitialBoundaries(configuration.phase1.initBoundsLow,...
                            configuration.phase1.initBoundsUpp);
phase1.setFinalBoundaries(configuration.phase1.finalBoundsLow,...
                          configuration.phase1.finalBoundsUpp);

%% Add cost function
problem.addNewParameterCost(FinalTime);

%% Solve problem
problem.Solve();

end
