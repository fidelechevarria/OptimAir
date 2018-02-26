function [x,fval,exitflag,output,population,score] = optimTraj_genetic(WP,hdngData)

close all

xLast = []; % Last place optimTraj was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

%% Define parameters
[numOfWaypoints,~] = size(WP.north);
nvars = (numOfWaypoints-1)*3;
PopulationSize = 40;
FunctionTolerance = 1e-3;
LB = [];
UB = [];

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

%% Start with the default options
options = gaoptimset;

%% Modify options setting
options = gaoptimset(options,'PopulationSize', PopulationSize);
options = gaoptimset(options,'Display', 'off');
options = gaoptimset(options,'PlotFcns', {  @gaplotbestf @gaplotscores });
options = gaoptimset(options,'TolFun' , FunctionTolerance);

%% Run optimizitation using Genetic Algorithm
[x,fval,exitflag,output,population,score] = ...
ga(optimTraj_fun,nvars,[],[],[],[],LB,UB,optimTraj_cfun,[],options);

%% Show optimization results
automaticFGlaunchIsActivated = 0;
optimTraj_results(x,WP,automaticFGlaunchIsActivated);

%% Display info
disp(char('',output.message)); % Display the reason why GA stopped iterating
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point FP time: ','',num2str(fval),'')); % Display solution's optimized FP time

end