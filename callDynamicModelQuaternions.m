
function optimizeControl()
 
    tic

    % Set options for pattern search
    options = psoptimset('Display','off','PlotFcn',{[]},...
        'UseParallel', false, 'CompletePoll', 'on','CompleteSearch','on',...
        'SearchMethod',@searchlhs);

    % Run the optimization
    [x,fval] = patternsearch(@objFcn,[0 0 0],[],[],[],[],...
        [-1000 -1000 -1000],[1000 1000 1000],[],options);

    toc

end

function [distance] = objFcn(vars)
    
    [Q_init] = eul2quat([0 0 0],'ZYX');

    control_vars.alpha_dot = vars(1);
    control_vars.delta_dot = vars(2);
    control_vars.p = vars(3);

    old_state.T = 5000;
    old_state.V = 90;
    old_state.alpha = 0.1;
    old_state.q0 = Q_init(1);
    old_state.q1 = Q_init(2);
    old_state.q2 = Q_init(3);
    old_state.q3 = Q_init(4);
    old_state.x = 0;
    old_state.y = 0;
    old_state.h = 0;
    
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
    
    dt = 0.1;
    
    [new_state] = dynamicModelQuaternions(control_vars, old_state, params, dt);
    [euler] = quat2eul([old_state.q0 old_state.q1 old_state.q2 old_state.q3],'ZYX');
    
    obj_state.x = 9;
    obj_state.y = 0;
    obj_state.h = 0;
    
    distance = sqrt((new_state.x-obj_state.x)^2+(new_state.y-obj_state.y)^2+(new_state.h-obj_state.h)^2);
     
end




function [new_state] = dynamicModelQuaternions(control_vars, old_state, params, dt)
    
    alpha_dot = control_vars.alpha_dot;
    T_dot = control_vars.delta_dot;
    p = control_vars.p;
    
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