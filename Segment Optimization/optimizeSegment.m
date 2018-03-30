
function optimizeSegment()

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
    problem.bounds.initialTime.low = 0;
    problem.bounds.initialTime.upp = 0;
    problem.bounds.finalTime.low = 0.5;
    problem.bounds.finalTime.upp = 100;

    problem.bounds.state.low = [0; 40; -0.5; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6];
    problem.bounds.state.upp = [9000; 1e6; 0.5; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6];
    problem.bounds.initialState.low = [4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0];
    problem.bounds.initialState.upp = [4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0];
    problem.bounds.finalState.low = [0; 40; -0.5; 0; 0; 0; 1; 0; 0; 50];
    problem.bounds.finalState.upp = [9000; 1e6; 0.5; 0; 0; 0; 1; 0; 0; 50];

    problem.bounds.control.low = [-10; -9000; -7.3];
    problem.bounds.control.upp = [10; 9000; 7.3];

    % Guess at the initial trajectory
    problem.guess.time = [0 20];
    problem.guess.state = [[4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0]...
                           [5000; 100; 0.1; 0; 0; 0; 1; 0; 0; 50]];
    problem.guess.control = zeros(3,2);

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
    soln = optimTraj(problem);

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

function states_dot = dynamics(states,controls,params)

    % States
    T = states(1,:);
    V = states(2,:);
    alpha = states(3,:);
    q0 = states(4,:);
    q1 = states(5,:);
    q2 = states(6,:);
    q3 = states(7,:);
    % x = states(8,:); %Not used in dynamics
    % y = states(9,:); %Not used in dynamics
    % h = states(10,:); %Not used in dynamics

    % Controls
    alpha_dot = controls(1,:);
    T_dot = controls(2,:);
    p = controls(3,:);

    % Constant parameters
    m = params.m;
    g = params.g;
    rho = params.rho;
    S = params.S;
    Clalpha = params.Clalpha;
    K = params.K;
    Cd0 = params.Cd0;

    % Dynamic model
    Cl = Clalpha.*alpha;
    L = 0.5.*rho.*V.^2.*S.*Cl;
    D = 0.5.*rho.*V.^2.*S.*(Cd0+K.*Cl.^2);
    q = (T.*sin(alpha)+L-m.*g.*(q0.^2-q1.^2-q2.^2+q3.^2))./(m.*V);
    r = (2.*m.*g.*(q0.*q1+q2.*q3))./(m.*V);
    V_dot = (T.*cos(alpha)-D+2.*m.*g.*(q1.*q3-q0.*q2))./m;
    q0_dot = -0.5.*(q1.*p+q2.*q+q3.*r);
    q1_dot =  0.5.*(q0.*p+q2.*r-q3.*q);
    q2_dot =  0.5.*(q0.*q+q3.*p-q1.*r);
    q3_dot =  0.5.*(q0.*r+q1.*q-q2.*p);
    x_dot = V.*(q0.^2+q1.^2-q2.^2-q3.^2);
    y_dot = 2.*V.*(q0.*q3+q1.*q2);
    h_dot = 2.*V.*(q0.*q2-q1.*q3);

    states_dot = [T_dot;V_dot;alpha_dot;q0_dot;q1_dot;q2_dot;q3_dot;x_dot;y_dot;h_dot];

end

function [c, ceq] = pathConstraint(x,u)
% [c, ceq] = pathConstraint(z)
%
% Computes the path constraint
  
c = [-x(10)];
ceq = [];  

end


