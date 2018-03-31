
function [trajectory] = optimizeSegment(segment)

    % Physical parameters
    p.m = 750;
    p.g = 9.81;
    p.rho = 1.225;
    p.S = 9.1;
    p.Clalpha = 4;
    p.K = 0.1779;
    p.Cd0 = 0.15;

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
    problem.options(1).trapezoid.nGrid = 60;
    problem.options(1).nlpOpt.MaxIter = 800;

    % problem.options(2).method = 'hermiteSimpson';
    % problem.options(2).hermiteSimpson.nSegment = 15;
    % problem.options(2).nlpOpt.MaxIter = 400;

    % Solve the problem
    soln = optimTrajDirCol(problem);

    % Unpack the simulation
    trajectory.segmentSize = 250;
    trajectory.time = linspace(soln(end).grid.time(1), soln(end).grid.time(end), trajectory.segmentSize);
    trajectory.states = soln(end).interp.state(trajectory.time);
    trajectory.controls = soln(end).interp.control(trajectory.time);

    % Plots:

    % Show the error in the collocation constraint between grid points:
    if 0;%strcmp(soln(end).problem.options.method,'trapezoid') || strcmp(soln(end).problem.options.method,'hermiteSimpson')
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

end


