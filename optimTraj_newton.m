function [x,fval,exitflag,output,lambda,grad,hessian] = optimTraj_newton(x0,WP,hdngData)

close all

xLast = []; % Last place optimTraj was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

%% Define parameters
[numOfWaypoints,~] = size(WP);
nvars = 2*numOfWaypoints;
PopulationSize = 40;
FunctionTolerance = 1e-3;
aprox_WP_azimuth = hdngData; % [degrees]
semiAngularThreshold = 10*ones(1,numOfWaypoints);
LB = [aprox_WP_azimuth-semiAngularThreshold zeros(1,nvars/2)];
UB = [aprox_WP_azimuth+semiAngularThreshold zeros(1,nvars/2)];
for i = 1:nvars/2
    index = nvars/2 + i;
    if (aprox_WP_azimuth(i) > 180) 
        LB(index) = - inf;
        UB(index) = 0;
    else
        LB(index) = 0;
        UB(index) = inf;
    end
end

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

%% Enter parameters for Newton-Raphson optimization
%x0 = [3.611665 59.22286 180.6453 304.6692 338.714 42.64582 65.10284 -18.8209 -47.46095 -27.21514]; % Initial point

%% Start Newton-Raphson optimization with the default options
options = optimoptions('fmincon');

%% Modify optimization options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcns', {  @optimplotx @optimplotfval });
options = optimoptions(options,'Diagnostics', 'off');

%% Run optimization
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(optimTraj_fun,x0,[],[],[],[],LB,UB,optimTraj_cfun,options);

%% Display final results
disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value

end