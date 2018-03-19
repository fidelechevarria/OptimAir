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

    %% Start Newton-Raphson optimization with the default options
    options = optimoptions('fmincon');

    %% Modify optimization options setting
    options = optimoptions(options,'Display', 'off');
    options = optimoptions(options,'PlotFcns', {  @optimplotx @optimplotfval @optimplotfunccount });
    options = optimoptions(options,'Diagnostics', 'off');

    %% Run optimization
    [x,fval,exitflag,output,lambda,grad,hessian] = ...
    fmincon(optimTraj_fun,IP,[],[],[],[],LB,UB,optimTraj_cfun,options);

    %% Display final results
    disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
    disp(char('','Last point: ','',num2str(x'))); % Display solution
    disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value

    %% Output function for plotting
    function stop = optimTraj_plotFcn(~,optimValues,state,varargin)

        stop = false;
        switch state
            case 'init'
                % clean up plot from a previous run, if any
                if ~isempty(plotfunc)
                    delete(plotfunc);
                end
            case 'iter'
                if optimValues.iteration == 0
                    % The 'iter' case is  called during the zeroth iteration,
                    % but it has values that were empty during the 'init' case
                    % 3D Graphical representation
                        f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
                        movegui(f1,'northwest') % Move the GUI to the center of the screen.
                        hold on
                        scatter3(complete_north,complete_east,complete_up,9,'r','filled')
                        scatter3(WP.north,WP.east,WP.up,9,'b','filled')
                        plot3(smooth_north,smooth_east,smooth_up)
                        hold off
                        grid
                        title('Trajectory approximation')
                        [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
                        axis equal
                        axis vis3d % Lock aspect ratio of axes
                        view(-45,45); % Azimuth and elevation of initial view (degrees)
                        xlabel('North')
                        ylabel('East')
                        zlabel('Up')
                    plotfunc = plot(optimValues.iteration,optimValues.funccount,'kd', ...
                        'MarkerFaceColor',[1 0 1]);
                    title(getString(message('MATLAB:funfun:optimplots:TitleTotalFunctionEvaluations',optimValues.funccount)),'interp','none');
                    xlabel(getString(message('MATLAB:funfun:optimplots:LabelIteration')),'interp','none');
                    ylabel(getString(message('MATLAB:funfun:optimplots:LabelEvaluations')),'interp','none');
                    set(plotfunc,'Tag','optimplotfunccount');
                else
                    % Not the zeroth iteration
                    totalFuncCount = sum(get(plotfunc,'Ydata'));
                    newX = [get(plotfunc,'Xdata') optimValues.iteration];
                    newY = [get(plotfunc,'Ydata') optimValues.funccount-totalFuncCount];
                    set(plotfunc,'Xdata',newX, 'Ydata',newY);
                    set(get(gca,'Title'),'String',getString(message('MATLAB:funfun:optimplots:TitleTotalFunctionEvaluations',optimValues.funccount)));
                end
        end

    end

end