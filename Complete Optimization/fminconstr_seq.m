function [x,fval,exitflag,output,lambda,grad,hessian] = fminconstr_seq(IP,WP,index,hdngData)

xLast = []; % Last place optimTraj was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

%% Define parameters

IP_seq = [IP(index) IP(index+numel(IP)/2)];

fun = @objfun; % the objective function, nested below
cfun = @constr; % the constraint function, nested below

    function y = objfun(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = time_seq(x,IP,WP,index);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = constr(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = time_seq(x,IP,WP,index);
            xLast = x;
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

    function stop = plot(x,optimValues,state,varargin) % custom plot function
        stop = customPlotFcn_seq(x,optimValues,state,varargin,IP,WP,index);
    end

%% Start Newton-Raphson optimization with the default options
options = optimoptions('fmincon');

%% Modify optimization options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcns', { @plot }); %optimplotfval optimplotfunccount
options = optimoptions(options,'Diagnostics', 'off');

%% Run optimization
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(fun,IP_seq,[],[],[],[],[],[],cfun,options);

end