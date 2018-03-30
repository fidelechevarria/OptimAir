function [x,fval] = multiStartParallel(WP)

    xLast = []; % Last place optimTraj was called
    time = []; % Use for objective at xLast

    % Define parameters
    IP = [-70 -180 -260 -450 -500 -400 -310 -200.0... % North initial points
                100 230 320 450 350 150 50 -150.0];   % East initial points
    margin = 250*ones(1,16);
    LB = IP - margin;
    UB = IP + margin;

    fun = @objfun; % the objective function, nested below

    function y = objfun(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [time] = totalTime(x,WP);
            xLast = x;
        end
        % Now compute objective function
        y = time;
    end

    % Set options for FMINCON
    TolFun = 0.01; % minimum distance between two separate objective function values
    TolX = 0.1; % minimum distance between two separate points
    options = optimset('Algorithm','interior-point','Disp','iter',...
        'TolFun',TolFun,'TolX',TolX,'Display', 'off',...
        'PlotFcns', { @customPlotFcn },'Diagnostics', 'off');

    % Create problem for MultiStart
    problem = createOptimProblem('fmincon','objective',fun,'x0',IP,...
                'lb',LB,'ub',UB,'options',options);

    % Make a MultiStart object        
    ms = MultiStart;

    % Tell MultiStart to use Parallel Computing
    ms.UseParallel = 'always';

    % Open up MATLAB Pool
    poolObj = parpool;

    % Run the optimization
    [x,fval,exitflag,output,solutions] = run(ms, problem, 10);

    % Close the MATLAB Pool
    delete(poolObj);

    % Display final results
    disp(char('','Last point: ','',num2str(x'))); % Display solution
    disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value

end
