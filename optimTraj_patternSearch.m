function [x,fval] = optimTraj_patternSearch(WP,hdngData)

xLast = []; % Last place optimTraj was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

%% Define parameters
IP = [-70 -180 -260 -450 -500 -400 -310 -200.0... % North initial points
            100 230 320 450 350 150 50 -150.0];   % East initial points
margin = 250*ones(1,16);
LB = IP - margin;
UB = IP + margin;

optimTraj_fun = @optimTraj_objfun; % the objective function, nested below
optimTraj_cfun = @optimTraj_constr; % the constraint function, nested below

    function y = optimTraj_objfun(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = optimTraj_time(x,WP);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = optimTraj_constr(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = optimTraj_time(x,WP);
            xLast = x;
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

%% Set options for pattern search
options = psoptimset('Display','iter','PlotFcn',@psplotbestf,...
    'UseParallel', true, 'CompletePoll', 'on');

%% Open up MATLAB Pool
poolObj = parpool;

%% Run the optimization
[x,fval] = patternsearch(optimTraj_fun,IP,[],[],[],[],LB,UB,optimTraj_cfun,options);

%% Close the MATLAB Pool
delete(poolObj);

%% Show optimization results
automaticFGlaunchIsActivated = 0;
optimTraj_results(x,WP,automaticFGlaunchIsActivated);

%% Display info
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point FP time: ','',num2str(fval),'')); % Display solution's optimized FP time

end

