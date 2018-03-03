function [x,fval] = optimTraj_multiStartParallel(WP,hdngData)

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

%% Set options for FMINCON
TolFun = 0.01; % minimum distance between two separate objective function values
TolX = 0.1; % minimum distance between two separate points
options = optimset('Algorithm','interior-point','Disp','iter',...
    'TolFun',TolFun,'TolX',TolX);

%% Create problem for MultiStart
problem = createOptimProblem('fmincon','objective',optimTraj_fun,'x0',IP,...
            'lb',LB,'ub',UB,'nonlcon',optimTraj_cfun,'options',options);

%% Make a MultiStart object        
ms = MultiStart;

%% Tell MultiStart to use Parallel Computing
ms.UseParallel = 'always';

%% Open up MATLAB Pool
poolObj = parpool;

%% Run the optimization
[x,fval,exitflag,output,solutions] = run(ms, problem, 64);

%% Close the MATLAB Pool
delete(poolObj);

%% Display final results
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value

end
