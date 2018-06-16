
function problem = optimizeSegmentPointMassModel(boundaries,guess,configuration)

%% Create States
states = [falcon.State('V',configuration.dynamics.minVel,configuration.dynamics.maxVel,0.005);
		  falcon.State('q0',-1,1,1);
          falcon.State('q1',-1,1,1);
	      falcon.State('q2',-1,1,1);
          falcon.State('q3',-1,1,1);
          falcon.State('x',configuration.dynamics.minPosNorth,configuration.dynamics.maxPosNorth,1e-3);
          falcon.State('y',configuration.dynamics.minPosEast,configuration.dynamics.maxPosEast,1e-3);
          falcon.State('h',configuration.dynamics.minAlt,configuration.dynamics.maxAlt,1e-3)];

%% Create Controls
controls = [falcon.Control('alpha',configuration.dynamics.minAlpha,configuration.dynamics.maxAlpha,1);
		    falcon.Control('T',0,configuration.dynamics.maxThrust,1e-3);
            falcon.Control('p',-configuration.dynamics.max_p,configuration.dynamics.max_p,0.1)];
           
%% Create model outputs
modeloutputs = [falcon.Output('q');
                falcon.Output('r');
                falcon.Output('V_dot')];
        
%% Create phase time
FinalTime = falcon.Parameter('FinalTime',guess.time(end),...
    configuration.options.minSegmentTime,configuration.options.maxSegmentTime,0.1);

%% Create complete problem
problem = falcon.Problem('optimTraj');

%% Set optimization parameters
problem.setMajorIterLimit(configuration.options.majIterLim);
% problem.setMajorFeasTol(1e-3);
% problem.setMajorOptTol(1e-14);

%% Specify discretization
tau = linspace(0,1,configuration.options.discretizationPoints);

%% Create Model
mdl = falcon.SimulationModelBuilder('pointMassModel', states, controls);

% Add constants
mdl.addConstant('m',configuration.dynamics.m); % kg
mdl.addConstant('g',configuration.dynamics.g); % m/s^2
mdl.addConstant('rho',configuration.dynamics.rho); % kg/m^3
mdl.addConstant('S',configuration.dynamics.S); % m^2
mdl.addConstant('Cl0',configuration.dynamics.Cl0);
mdl.addConstant('Clalpha',configuration.dynamics.Clalpha);
mdl.addConstant('Cd0',configuration.dynamics.Cd0);
mdl.addConstant('K',configuration.dynamics.K);
mdl.addConstant('Cdp',configuration.dynamics.Cdp);
mdl.addConstant('windVelocityEarth',configuration.dynamics.windVelocityEarth); % m/s

% Aerodynamic forces
mdl.addSubsystem(@dyn_forces,...
    {'Clalpha','rho','S','Cd0','Cl0','Cdp','K','alpha','V','p'},... % Inputs
    {'L','D'}); % Outputs

% Angular velocities
mdl.addSubsystem(@dyn_angVels,...
    {'m','g','T','V','q0','q1','q2','q3','L','alpha'},... % Inputs
    {'q','r'}); % Outputs

% Velocity derivative
mdl.addSubsystem(@dyn_velDot,...
    {'m','g','T','alpha','D','q0','q1','q2','q3'},... % Inputs
    {'V_dot'}); % Outputs

% Quaternion derivatives
mdl.addSubsystem(@dyn_quatDot,...
    {'q0','q1','q2','q3','p','q','r'},... % Inputs
    {'q0_dot','q1_dot','q2_dot','q3_dot'}); % Outputs

% Position derivatives
mdl.addSubsystem(@dyn_positionDot,...
    {'q0','q1','q2','q3','V','windVelocityEarth'},... % Inputs
    {'x_dot','y_dot','h_dot'}); % Outputs

% Set the variable names of the derivatives to tell FALCON the correct
% outputs
mdl.setStateDerivativeNames('V_dot','q0_dot','q1_dot','q2_dot',...
    'q3_dot','x_dot','y_dot','h_dot');

% Add model outputs
mdl.setOutputs(modeloutputs);

% Build the Model evaluating the subsystem chain
mdl.Build();

%% Create pathConstraint

cnstr = falcon.PathConstraintBuilder('pathConstraintBuild',modeloutputs,states,controls);

% Add constants
cnstr.addConstant('g',configuration.dynamics.g); % m/s^2
cnstr.addConstant('SL_m',configuration.SL.m)
cnstr.addConstant('SL_n',configuration.SL.n)

% Transform gravity vector to body axes
cnstr.addSubsystem(@dyn_gravBody,...
    {'q0','q1','q2','q3','g'},... % Inputs
    {'gx','gy','gz'}); % Outputs

% Calculate body accelerations
cnstr.addSubsystem(@dyn_accelsBody,...
    {'gx','gy','gz','q','r','V','V_dot'},... % Inputs
    {'Ax','Ay','Az'}); % Outputs

% Calculate acceleration norm
cnstr.addSubsystem(@dyn_accelNorm,...
    {'Ax','Ay','Az'},... % Inputs
    {'A_norm'}); % Outputs

% Calculate Safety Line constraint
cnstr.addSubsystem(@dyn_safetyLine,...
    {'x','y','SL_m','SL_n'},... % Inputs
    {'SL'}); % Outputs

% Set the variable names of the constraints
cnstr.setConstraintValueNames('A_norm','Az','SL');

% Build the constraints evaluating the subsystem chain
cnstr.Build();

%% Create phase

% Phase
phase1 = problem.addNewPhase(@pointMassModel,states,tau,0,FinalTime);

% State grid
phase1.StateGrid.setValues(guess.time,guess.states,'Realtime',true);

% Control grid
controlgrid = phase1.addNewControlGrid(controls,tau);
controlgrid.setValues(guess.time,guess.controls,'Realtime',true);

% Set boundary condition
phase1.setInitialBoundaries(boundaries.phase1.initBoundsLow,...
                            boundaries.phase1.initBoundsUpp);
phase1.setFinalBoundaries(boundaries.phase1.finalBoundsLow,...
                          boundaries.phase1.finalBoundsUpp);

% Path Constraint

maxms = configuration.dynamics.maxG * configuration.dynamics.g;
maxms_neg = configuration.dynamics.maxG_neg * configuration.dynamics.g;
if configuration.SL.active == true
    if -configuration.SL.n/configuration.SL.m > 0
        % Safety Line is West
        SL_low = 0;
        SL_high = inf;
    else
        % Safety Line is East
        SL_low = -inf;
        SL_high = 0;
    end
else
    SL_low = -inf;
    SL_high = inf;
end
constraint_vals = [falcon.Constraint('A_norm',-maxms,maxms);
                   falcon.Constraint('A_z',-maxms,maxms_neg);
                   falcon.Constraint('SL',SL_low,SL_high)];
phase1.addNewPathConstraint(@pathConstraintBuild, constraint_vals);

% Set model outputs
phase1.Model.setModelOutputs(modeloutputs);

%% Add cost function
problem.addNewParameterCost(FinalTime);

%% Solve problem
problem.Solve();

end
