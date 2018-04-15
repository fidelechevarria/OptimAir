
function problem = optimizeTrajectory(configuration)

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

% State grid
% phase1.StateGrid.setValues([0 4.63186379666037 9.26372759332075],...
%     [[5000.00000000000;79.9999999999999;0.100000000000001;-0.500000000000030;-7.96173597228149e-15;-1.75565327877039e-15;0.866025403784482;-3.25558263224744e-12;9.00702850838653e-12;20],...
%     [8999.12485958459;47.5682835438019;0.192096983706471;0.771894959880510;-0.444811714355299;-0.336778657137903;0.304796402094414;-63.2644824518742;48.8184903964936;90.3718845174613],...
%     [8879.51057728200;75.2802702725374;-0.0737225522791648;0.882947592858952;-1.13662826198362e-15;-1.17022368407119e-15;0.469471562785909;135.071808203098;267.248773317919;20.0000000000010]],...
%     'Realtime',true);

% Control grid
controlgrid = phase1.addNewControlGrid(controls,tau);
% controlgrid.setValues([0 4.63186379666037 9.26372759332075],...
%     [6.11404198094083;8996.09796581553;-1.12446649563586],...
%     [-1.85030452083568;-5034.84036415976;1.08963238596043],...
%     [-1.42522333606845;-5381.48157958598;-1.08880958921222],...
%     'Realtime',true);

% Set boundary condition
phase1.setInitialBoundaries(configuration.phase1.initBoundsLow,...
                            configuration.phase1.initBoundsUpp);
phase1.setFinalBoundaries(configuration.phase1.finalBoundsLow,...
                          configuration.phase1.finalBoundsUpp);

% Path Constraint
pathconstraint = falcon.Constraint('quat_const', 0, 0);
phase1.addNewPathConstraint(@pathConstraints, pathconstraint);

%% Add cost function
problem.addNewParameterCost(FinalTime);

%% Solve problem
problem.Solve();

end
