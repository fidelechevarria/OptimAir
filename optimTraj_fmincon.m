function [x,fval,exitflag,output,lambda,grad,hessian] = optimTraj_fmincon(x0,WP,hdngData)

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

    function stop = optimTraj_plot(x,optimValues,state,varargin) % custom plot function
        stop = optimTraj_customPlotFcn(x,optimValues,state,varargin,WP);
    end

%% Start Newton-Raphson optimization with the default options
options = optimoptions('fmincon');

%% Modify optimization options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcns', {  @optimplotx @optimplotfval...
    @optimplotfunccount @optimTraj_plot });
options = optimoptions(options,'Diagnostics', 'off');

%% Run optimization
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(optimTraj_fun,IP,[],[],[],[],LB,UB,optimTraj_cfun,options);

%% Display final results
disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
disp(char('','Last point: ','',num2str(x'))); % Display solution
disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value

end