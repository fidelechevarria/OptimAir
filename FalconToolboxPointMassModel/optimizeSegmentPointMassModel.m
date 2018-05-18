
function problem = optimizeSegmentPointMassModel(boundaries,guess,configuration)

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
FinalTime = falcon.Parameter('FinalTime',guess.time(end),1,50,0.1);

%% Create complete problem
problem = falcon.Problem('optimTraj');

%% Set optimization parameters
problem.setMajorIterLimit(configuration.majIterLim);
% problem.setMajorFeasTol(1e-3);
% problem.setMajorOptTol(1e-14);

%% Specify discretization
tau = linspace(0,1,configuration.discretizationPoints);

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
pathconstraint = falcon.Constraint('quat_const', 0, 0);
phase1.addNewPathConstraint(@pathConstraintsPointMassModel, pathconstraint);

%% Add cost function
problem.addNewParameterCost(FinalTime);

%% Solve problem
problem.Solve();

%% Create timeseries from solution
% [statesTS, controlTS, outputTS, statesdotTS, postprocessedTS] = problem.getTimeSeries();

% FG_x = timeseries(statesTS.Data(:,10), statesTS.Time);
% FG_y = timeseries(statesTS.Data(:,11), statesTS.Time);
% FG_alt = timeseries(-statesTS.Data(:,12), statesTS.Time);
% FG_roll = timeseries(statesTS.Data(:,4), statesTS.Time);
% FG_pitch = timeseries(statesTS.Data(:,5), statesTS.Time);
% FG_yaw = timeseries(statesTS.Data(:,6), statesTS.Time);
% FG_time = statesTS.Time(end);
% 
% assignin('base','FG_x',FG_x);
% assignin('base','FG_y',FG_y);
% assignin('base','FG_alt',FG_alt);
% assignin('base','FG_roll',FG_roll);
% assignin('base','FG_pitch',FG_pitch);
% assignin('base','FG_yaw',FG_yaw);
% assignin('base','FG_time',FG_time);
% 
% sim('trajectorySimulation.slx')

end