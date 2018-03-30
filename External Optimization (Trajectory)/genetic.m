function [x,fval,exitflag,output,population,score] = genetic(WP,hdngData)

xLast = []; % Last place optimTraj was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

%% Define parameters
[numOfWaypoints,~] = size(WP.north);
nvars = (numOfWaypoints-1)*3;
PopulationSize = 40;
FunctionTolerance = 1e-3;
IP = [-70 -180 -260 -450 -500 -400 -310 -200.0... % North initial points
            100 230 320 450 350 150 50 -150.0];   % East initial points
margin = 250*ones(1,16);
LB = IP - margin;
UB = IP + margin;

fun = @objfun; % the objective function, nested below
cfun = @constr; % the constraint function, nested below

    function y = objfun(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = time(x,WP);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = constr(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = time(x,WP);
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
ga(fun,nvars,[],[],[],[],LB,UB,cfun,[],options);

%% Show optimization results
automaticFGlaunchIsActivated = 0;
results(x,WP,automaticFGlaunchIsActivated);

%% Display info
disp(char('',output.message)); % Display the reason why GA stopped iterating
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point FP time: ','',num2str(fval),'')); % Display solution's optimized FP time

end