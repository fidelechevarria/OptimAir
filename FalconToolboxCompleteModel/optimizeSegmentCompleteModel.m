
function problem = optimizeSegmentCompleteModel(boundaries,guess,configuration)

%% Create States
states = [falcon.State('vx',10,100,1e-2);
          falcon.State('vy',-100,100,1e-2);
		  falcon.State('vz',-100,100,1e-2);
          falcon.State('roll',-4,4,1);
          falcon.State('pitch',-4,4,1);
          falcon.State('yaw',-4,4,1);
		  falcon.State('p',-8,8,1);
          falcon.State('q',-8,8,1);
	      falcon.State('r',-8,8,1);
          falcon.State('x',-2000,2000,1e-3);
          falcon.State('y',-2000,2000,1e-3);
          falcon.State('h',5,2000,1e-1)];

%% Create Controls
controls = [falcon.Control('da',-0.35,0.35,1);
		    falcon.Control('de',-0.35,0.35,1);
            falcon.Control('dr',-0.35,0.35,1);
            falcon.Control('dt',0,1,1)];
        
%% Create model outputs
modeloutputs = [falcon.Output('vx_dot');
                falcon.Output('vy_dot');
                falcon.Output('vz_dot')];
        
%% Create phase time
FinalTime = falcon.Parameter('FinalTime',guess.time(end),1,50,0.1);

%% Create complete problem
problem = falcon.Problem('OptimAir');

%% Set optimization parameters
problem.setMajorIterLimit(configuration.majIterLim);
% problem.setMajorFeasTol(1e-3);
% problem.setMajorOptTol(1e-14);

%% Specify discretization
tau = linspace(0,1,configuration.discretizationPoints);

%% Create Model
mdl = falcon.SimulationModelBuilder('dynamicModel', states, controls);

% Add constants
mdl.addConstant('m',750); % kg
mdl.addConstant('g',9.8056); % m/s^2
mdl.addConstant('rho',1.225); % kg/m^3
mdl.addConstant('S',9.84); % m^2
mdl.addConstant('Tmax',9000); % N
mdl.addConstant('b',7.87); % m
mdl.addConstant('c',1.25); % m
mdl.addConstant('Cd0',0.15); 
mdl.addConstant('Cda2',0.1);
mdl.addConstant('Cdb',0.15);
mdl.addConstant('Cyb',-0.4);
mdl.addConstant('Cyda',0);
mdl.addConstant('Cydr',0.19); 
mdl.addConstant('Cyp',0);
mdl.addConstant('Cyr',0.4);
mdl.addConstant('Cl0',0.1205);
mdl.addConstant('Cla',5.7);
mdl.addConstant('Cllb',-0.0002); 
mdl.addConstant('Cllda',0.33);
mdl.addConstant('Clldr',0.021);
mdl.addConstant('Cllp',-0.79);
mdl.addConstant('Cllr',0.075);
mdl.addConstant('Cmm0',0); 
mdl.addConstant('Cmma',-1.23);
mdl.addConstant('Cmmda',0);
mdl.addConstant('Cmmde',-1.1);
mdl.addConstant('Cmmdr',0);
mdl.addConstant('Cmmq',-7.34);
mdl.addConstant('Cnnb',0.21);
mdl.addConstant('Cnnda',-0.014);
mdl.addConstant('Cnndr',-0.11); 
mdl.addConstant('Cnnp',-0.024);
mdl.addConstant('Cnnr',-0.265);
mdl.addConstant('Ix',3531.9); % kg*m^2
mdl.addConstant('Iy',2196.4); % kg*m^2
mdl.addConstant('Iz',4887.7); % kg*m^2
mdl.addConstant('Ixz',0); % kg*m^3

% Aerodynamic angles
mdl.addSubsystem(@dyn_alphaBeta,...
    {'vx','vy','vz'},... % Inputs
    {'V','alpha','beta'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_eulTrigon,...
    {'roll','pitch','yaw'},... % Inputs
    {'cr','sr','cp','sp','tp','cy','sy'}); % Outputs

% Auxiliary coefficients
mdl.addSubsystem(@dyn_coeffsAux,...
    {'b','c','V'},... % Inputs
    {'coeffA','coeffB'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_dragCoeff,...
    {'Cd0','Cda2','Cdb','alpha','beta'},... % Inputs
    {'Cd'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_lateralForceCoeff,...
    {'Cyb','Cydr','Cyda','Cyp','Cyr','beta','dr','da','coeffA','p','r'},... % Inputs
    {'Cy'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_liftCoeff,...
    {'Cl0','Cla','alpha'},... % Inputs
    {'Cl'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_rollingMomentCoeff,...
    {'Cllb','Cllda','Clldr','Cllp','Cllr','beta','da','dr','coeffA','p','r'},... % Inputs
    {'Cll'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_pitchingMomentCoeff,...
    {'Cmm0','Cmma','Cmmde','Cmmdr','Cmmda','Cmmq','alpha','de','dr','da','coeffB','q'},... % Inputs
    {'Cmm'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_yawingMomentCoeff,...
    {'Cnnb','Cnnda','Cnndr','Cnnp','Cnnr','beta','da','dr','coeffA','p','r'},... % Inputs
    {'Cnn'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_forcesMomentsWind,...
    {'rho','V','S','b','c','Cd','Cy','Cl','Cll','Cmm','Cnn'},... % Inputs
    {'D','Y','L','LL','MM','NN'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_forcesBody,...
    {'alpha','beta','D','Y','L','Tmax','dt'},... % Inputs
    {'Xa','Ya','Za','Xt'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_velBodyDot,...
    {'vx','vy','vz','p','q','r','Xa','Xt','Ya','Za','g','m','sp','cp','sr','cr'},... % Inputs
    {'vx_dot','vy_dot','vz_dot'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_eulDot,...
    {'p','q','r','tp','sr','cr','cp'},... % Inputs
    {'roll_dot','pitch_dot','yaw_dot'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_angularRatesDot,...
    {'Ix','Iy','Iz','Ixz','p','q','r','LL','MM','NN'},... % Inputs
    {'p_dot','q_dot','r_dot'}); % Outputs

% Euler angles trigonometric relations
mdl.addSubsystem(@dyn_posDot,...
    {'vx','vy','vz','sr','cr','sp','cp','sy','cy'},... % Inputs
    {'x_dot','y_dot','h_dot'}); % Outputs

% Set the variable names of the derivatives to tell FALCON the correct
% outputs
mdl.setStateDerivativeNames('vx_dot','vy_dot','vz_dot','roll_dot','pitch_dot','yaw_dot',...
    'p_dot','q_dot','r_dot','x_dot','y_dot','h_dot');

% Add model outputs
mdl.setOutputs(modeloutputs);

% Build the Model evaluating the subsystem chain
mdl.Build();

%% Create pathConstraint

cnstr = falcon.PathConstraintBuilder('pathConstraintBuild_comp',modeloutputs,states,controls);

% Add constants
cnstr.addConstant('g',9.8056); % m/s^2

% Transform gravity vector to body axes
cnstr.addSubsystem(@dyn_gravBody_comp,...
    {'roll','pitch','g'},... % Inputs
    {'gx','gy','gz'}); % Outputs

% Calculate body accelerations
cnstr.addSubsystem(@dyn_accelsBody_comp,...
    {'gx','gy','gz','p','q','r','vx','vy','vz','vx_dot','vy_dot','vz_dot'},... % Inputs
    {'Ax','Ay','Az'}); % Outputs

% Calculate acceleration norm
cnstr.addSubsystem(@dyn_accelNorm_comp,...
    {'Ax','Ay','Az'},... % Inputs
    {'A_norm'}); % Outputs)

% Set the variable names of the constraints
cnstr.setConstraintValueNames('A_norm','Az');

% Build the constraints evaluating the subsystem chain
cnstr.Build();

%% Create phase

% Phase
phase1 = problem.addNewPhase(@dynamicModel,states,tau,0,FinalTime);

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
constraint_vals = [falcon.Constraint('max_accel_norm_comp',-100,100);
                   falcon.Constraint('max_accel_z_comp',-100,20)];
phase1.addNewPathConstraint(@pathConstraintBuild_comp, constraint_vals);

% Set model outputs
phase1.Model.setModelOutputs(modeloutputs);
                      
%% Add cost function
problem.addNewParameterCost(FinalTime);

%% Solve problem
problem.Solve();

%% Create timeseries from solution
[statesTS, controlTS, outputTS, statesdotTS, postprocessedTS] = problem.getTimeSeries();

FG_x = timeseries(statesTS.Data(:,10), statesTS.Time);
FG_y = timeseries(statesTS.Data(:,11), statesTS.Time);
FG_alt = timeseries(-statesTS.Data(:,12), statesTS.Time);
FG_roll = timeseries(statesTS.Data(:,4), statesTS.Time);
FG_pitch = timeseries(statesTS.Data(:,5), statesTS.Time);
FG_yaw = timeseries(statesTS.Data(:,6), statesTS.Time);
FG_time = statesTS.Time(end);

assignin('base','FG_x',FG_x);
assignin('base','FG_y',FG_y);
assignin('base','FG_alt',FG_alt);
assignin('base','FG_roll',FG_roll);
assignin('base','FG_pitch',FG_pitch);
assignin('base','FG_yaw',FG_yaw);
assignin('base','FG_time',FG_time);

% sim('trajectorySimulation.slx')

end
