
function [trajectory] = optimizeSegment(segment)

    % Physical parameters
    p.Tmax = 9000;
    p.Tmin = 0;
    p.m = 750;
    p.g = 9.81;
    p.rho = 1.225;
    p.S = 9.1;
    p.Clalpha = 1.3;
    p.K = 0.1779;
    p.Cd0 = 0.036;
    p.alpha_min = -0.5;
    p.alpha_max = 0.5;

    % User-defined dynamics and objective functions
    problem.func.dynamics = @(t,x,u)( dynamics(x,u,p) );
    problem.func.pathObj = @(t,x,u)( ones(size(t)) ); % Optimize total time
    problem.func.pathCst = @(t,x,u)(  pathConstraint(x,u)  );

    % Problem bounds
    problem.bounds = segment.bounds;

    % Guess at the initial trajectory
    problem.guess = segment.guess;

    % Select a solver:
%     problem.options(1).method = 'rungeKutta'; % Buen comportamiento en al
%                                               % menos "medium"
%     problem.options(1).defaultAccuracy = 'medium';

    problem.options(1).method = 'trapezoid';
    problem.options(1).trapezoid.nGrid = 15;
    problem.options(1).nlpOpt.MaxIter = 200;

    % problem.options(2).method = 'hermiteSimpson';
    % problem.options(2).hermiteSimpson.nSegment = 15;
    % problem.options(2).nlpOpt.MaxIter = 400;

    % Solve the problem
    soln = optimTrajDirCol(problem);

    % Unpack the simulation
    time = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 150);
    states = soln(end).interp.state(time);
    controls = soln(end).interp.control(time);

    % Plots:

    % Show the error in the collocation constraint between grid points:
    if strcmp(soln(end).problem.options.method,'trapezoid') || strcmp(soln(end).problem.options.method,'hermiteSimpson')
        % Then we can plot an estimate of the error along the trajectory
        figure(5); clf;

        % NOTE: the following commands have only been implemented for the direct
        % collocation(trapezoid, hermiteSimpson) methods, and will not work for
        % chebyshev or rungeKutta methods.
        cc = soln(end).interp.collCst(time);

        subplot(2,2,1);
        plot(time,cc(1,:))
        title('Collocation Error:   dx/dt - f(t,x,u)')
        ylabel('d/dt cart position')

        subplot(2,2,3);
        plot(time,cc(2,:))
        xlabel('time')
        ylabel('d/dt pole angle')

        idx = 1:length(soln(end).info.error);
        subplot(2,2,2); hold on;
        plot(idx,soln(end).info.error(1,:),'ko');
        title('State Error')
        ylabel('cart position')

        subplot(2,2,4); hold on;
        plot(idx,soln(end).info.error(2,:),'ko');
        xlabel('segment index')
        ylabel('pole angle');
    end

    %%%% Plot the state and control against time
    figure
    hold on
    plot3(states(8,:),states(9,:),states(10,:));
    hold off
    grid
    view(45,45)
    axis equal
    axis vis3d

end


