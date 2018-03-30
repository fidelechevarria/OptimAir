
function [state_history] = propagateDynamicModel(trajectory)

    dt = 0.05; % Model propagation at 20 Hz

    [Q_init] = eul2quat(deg2rad([120 0 0]),'ZYX'); % Initial euler angles

    % Initial conditions
    state.T = 5000; 
    state.V = 90;
    state.alpha = 0.1;
    state.q0 = Q_init(1);
    state.q1 = Q_init(2);
    state.q2 = Q_init(3);
    state.q3 = Q_init(4);
    state.x = 0;
    state.y = 0;
    state.h = 20;

    params.Tmax = 9000;
    params.Tmin = 0;
    params.m = 750;
    params.g = 9.81;
    params.rho = 1.225;
    params.S = 9.1;
    params.Clalpha = 1.3;
    params.K = 0.1779;
    params.Cd0 = 0.036;
    params.alpha_min = -0.5;
    params.alpha_max = 0.5;

    N = 500;
    space = linspace(trajectory.breaks(1),trajectory.breaks(end),N);
    discreteTraj = fnval(trajectory,space);

    xLast = []; % Last place optimTraj was called
    myf = []; % Use for objective at xLast
    myc = []; % Use for nonlinear inequality constraint
    myceq = []; % Use for nonlinear equality constraint

    obj = @objFcn_caller; % the objective function, nested below
    constr = @objFcn_constr_caller; % the constraint function, nested below

    function y = objFcn_caller(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = objFcn(x,discreteTraj,state,params,dt);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = objFcn_constr_caller(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = objFcn(x,discreteTraj,state,params,dt);
            xLast = x;
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

    % Set options for pattern search
    options = psoptimset('Display','off','PlotFcn',{[]},...
        'UseParallel', false, 'CompletePoll', 'on','CompleteSearch','on',...
        'SearchMethod',@searchlhs);

    state_history.x = [];
    state_history.y = [];
    state_history.h = [];
    state_history.V = [];
    state_history.alpha_dot = [];
    state_history.T_dot = [];
    state_history.p = [];
    state_history.alpha = [];
    state_history.T = [];
    state_history.roll = [];
    for i = 1:500
        % Run the optimization
        [x,fval] = patternsearch(obj,[0 0 0],[],[],[],[],...
            [-1000 -1000 -1000],[1000 1000 1000],constr,options);
        
        [state] = dynamicModelStep(x, state, params, dt);
        [euler] = quat2eul([state.q0 state.q1 state.q2 state.q3],'ZYX');
        state_history.x = [state_history.x state.x];
        state_history.y = [state_history.y state.y];
        state_history.h = [state_history.h state.h];
        state_history.V = [state_history.V state.V];
        state_history.alpha_dot = [state_history.alpha_dot x(1)];
        state_history.T_dot = [state_history.T_dot x(2)];
        state_history.p = [state_history.p x(3)];
        state_history.alpha = [state_history.alpha state.alpha];
        state_history.T = [state_history.T state.T];
        state_history.roll = [state_history.roll euler(3)];
    end
    
end

function [f,c,ceq] = objFcn(vars,discreteTraj,initial_state,params,dt)
        
    [final_state] = dynamicModelStep(vars, initial_state, params, dt);
    
    [distanceToCurve,closestPointInCurve] = distance2curve(discreteTraj,[final_state.x final_state.y final_state.h]);
    
    distanceProjectionAlongCurve = sqrt((closestPointInCurve.x-initial_state.x)^2+...
                                        (closestPointInCurve.y-initial_state.y)^2+...
                                        (closestPointInCurve.z-initial_state.h)^2);
    
    objVal = distanceToCurve;%; - distanceProjectionAlongCurve; % Minimize distance to curve but maximize distance projection along curve
     
    % Function outputs
    f = objVal; % Value to minimize
    c = [ ]; % if >0 params are not a valid solution
    ceq = [ ]; % if =0 params are not a valid solution
    
end

function [new_state] = dynamicModelStep(control_vars, old_state, params, dt)
    
    alpha_dot = control_vars(1);
    T_dot = control_vars(2);
    p = control_vars(3);
    
    T = old_state.T;
    V = old_state.V;
    alpha = old_state.alpha;
    q0 = old_state.q0;
    q1 = old_state.q1;
    q2 = old_state.q2;
    q3 = old_state.q3;
    x = old_state.x;
    y = old_state.y;
    h = old_state.h;
    
    Tmax = params.Tmax;
    Tmin = params.Tmin;
    m = params.m;
    g = params.g;
    rho = params.rho;
    S = params.S;
    Clalpha = params.Clalpha;
    K = params.K;
    Cd0 = params.Cd0;
    alpha_min = params.alpha_min;
    alpha_max = params.alpha_max;
    
    T = max(min(T + T_dot*dt, Tmax), Tmin);
    alpha = max(min(alpha + alpha_dot*dt, alpha_max), alpha_min);
    Cl = Clalpha*alpha;
    L = 0.5*rho*V^2*S*Cl;
    D = 0.5*rho*V^2*S*(Cd0+K*Cl^2);
    q = (T*sin(alpha)+L-m*g*(q0^2-q1^2-q2^2+q3^2))/(m*V);
    r = (2*m*g*(q0*q1+q2*q3))/(m*V);
    V_dot = (T*cos(alpha)-D+2*m*g*(q1*q3-q0*q2))/m;
    q0_dot = -0.5*(q1*p+q2*q+q3*r);
    q1_dot =  0.5*(q0*p+q2*r-q3*q);
    q2_dot =  0.5*(q0*q+q3*p-q1*r);
    q3_dot =  0.5*(q0*r+q1*q-q2*p);
    V = V + V_dot*dt;
    q0 = q0 + q0_dot*dt;
    q1 = q1 + q1_dot*dt;
    q2 = q2 + q2_dot*dt;
    q3 = q3 + q3_dot*dt;
    qnorm = sqrt(q0^2+q1^2+q2^2+q3^2);
    q0 = q0 / qnorm;
    q1 = q1 / qnorm;
    q2 = q2 / qnorm;
    q3 = q3 / qnorm;
    x_dot = V*(q0^2+q1^2-q2^2-q3^2);
    y_dot = 2*V*(q0*q3+q1*q2);
    h_dot = 2*V*(q0*q2-q1*q3);
    x = x + x_dot*dt;
    y = y + y_dot*dt;
    h = h + h_dot*dt;
    
    new_state.T = T;
    new_state.V = V;
    new_state.alpha = alpha;
    new_state.q0 = q0;
    new_state.q1 = q1;
    new_state.q2 = q2;
    new_state.q3 = q3;
    new_state.x = x;
    new_state.y = y;
    new_state.h = h;
    
end